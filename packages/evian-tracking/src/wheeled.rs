//! Wheeled odometry.

use evian_math::{Angle, IntoAngle, Vec2};

use alloc::rc::Rc;
use core::{
    cell::RefCell,
    f64::consts::{PI, TAU},
};
use vexide::{
    devices::smart::Motor,
    prelude::{Task, sleep, spawn},
    time::Instant,
};

use crate::{Gyro, sensor::RotarySensor};
use crate::{TracksForwardTravel, TracksHeading, TracksPosition};

use super::TracksVelocity;

// MARK: Tracking Wheel

/// A wheel attached to a rotary sensor for position tracking.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackingWheel<T: RotarySensor> {
    /// Rotary sensor for measuring the wheel's travel.
    pub sensor: T,

    /// Diameter of the wheel.
    pub wheel_diameter: f64,

    /// Signed offset from the drivetrain's center of rotation.
    ///
    /// Negative `offset` implies that the wheel is left or behind
    /// the center of rotation.
    pub offset: f64,

    /// External gearing of the wheel.
    ///
    /// Used as a multiplier when determining wheel travel.
    pub gearing: Option<f64>,
}

impl<T: RotarySensor> TrackingWheel<T> {
    /// Creates a new tracking wheel with the given parameters.
    ///
    /// # Parameters
    ///
    /// * `sensor` - The rotary sensor to read wheel rotation from.
    /// * `wheel_diameter` - The diameter of the wheel in linear units.
    /// * `offset` - Distance from wheel to robot's center of rotation.
    /// * `gearing` - Optional gear ratio between sensor and wheel (use None for 1:1 if ungeared).
    pub const fn new(sensor: T, wheel_diameter: f64, offset: f64, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            wheel_diameter,
            offset,
            gearing,
        }
    }

    /// Calculates the total linear distance traveled by this wheel.
    ///
    /// This method uses the wheel's diameter and gear ratio to convert
    /// sensor rotations into linear distance traveled.
    ///
    /// # Errors
    ///
    /// Returns an error if [`RotarySensor::position`] fails.
    pub fn travel(&self) -> Result<f64, T::Error> {
        let wheel_circumference = self.wheel_diameter * PI;

        Ok(self.sensor.position()?.as_revolutions()
            * self.gearing.unwrap_or(1.0)
            * wheel_circumference)
    }
}

enum HeadingError<T: RotarySensor> {
    /// Gyro failed for whatever reason (we don't care exactly what happened,
    /// since all gyro failures are potentially fatal to tracking).
    ///
    /// A backup angle computed from two parallel forward tracking is provided,
    /// if such wheels are available.
    Imu(Option<Angle>),

    /// [`RotarySensor`] failed to return position data required to compute heading
    /// from a pair of parallel forward tracking wheels.
    RotarySensor(T::Error),
}

/// Generic tracking data returned by [`ParallelWheelTracking`] and [`PerpendicularWheelTracking`].
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    position: Vec2<f64>,
    raw_heading: Angle,
    heading_offset: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

// MARK: Tracking Implementation

/// Tracking system that uses wheels to track position and orientation.
#[derive(Debug)]
pub struct WheeledTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl WheeledTracking {
    /// Creates a new wheeled tracking system.
    pub fn new<
        T: RotarySensor + 'static,
        U: RotarySensor + 'static,
        G: Gyro + 'static,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        origin: impl Into<Vec2<f64>>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        mut gyro: Option<G>,
    ) -> Self {
        const FORWARD_TRACKER_OFFSET_TOLERANCE: f64 = 0.5;

        const {
            assert!(
                NUM_FORWARD > 0,
                "Wheeled tracking requires at least one forward tracking wheel."
            );
        }

        assert!(
            NUM_FORWARD >= 2 || gyro.is_some(),
            "Wheeled tracking requires either a Gyro or two parallel forward tracking wheels to determine robot orientation."
        );

        // Locate two parallel tracking wheels with roughly the same absolute offset from the
        // center of tracking. We use these for our backup wheeled heading calculations.
        let mut parallel_forward_indicies = None;
        if NUM_FORWARD >= 2 {
            for i in 0..NUM_FORWARD {
                for j in (i + 1)..NUM_FORWARD {
                    let i_wheel = &forward_wheels[i];
                    let j_wheel = &forward_wheels[j];

                    // Check if their offsets are acceptable enough
                    if (i_wheel.offset + j_wheel.offset).abs() <= FORWARD_TRACKER_OFFSET_TOLERANCE {
                        parallel_forward_indicies = Some(if i_wheel.offset < j_wheel.offset {
                            (i, j)
                        } else {
                            (j, i)
                        });
                    }
                }
            }
        }

        assert!(
            gyro.is_some() || parallel_forward_indicies.is_some(),
            "No gyro provided or viable tracking wheels available to determine robot orientation."
        );

        let initial_forward_wheel_data = forward_wheels
            .each_ref()
            .map(|wheel| wheel.travel().map(|travel| (travel, wheel.offset)));
        let initial_sideways_wheel_data = sideways_wheels
            .each_ref()
            .map(|wheel| wheel.travel().map(|travel| (travel, wheel.offset)));
        let initial_raw_heading = match Self::compute_raw_heading(
            gyro.as_ref(),
            parallel_forward_indicies.map(|(left_index, right_index)| {
                (&forward_wheels[left_index], &forward_wheels[right_index])
            }),
        ) {
            Ok(heading) => heading,
            Err(HeadingError::Imu(wheel_heading)) => {
                gyro = None;
                // NOTE: This returning `None` means that there's no real point in spawning the task since
                // the gyro disconnected, but we'll leave that to the task to figure out, since there's an
                // early return condition in the loop if this occurs and we don't want to have to handle
                // errors in this specific function.
                wheel_heading.unwrap_or_default()
            }
            _ => Angle::default(),
        };
        let initial_forward_travel = {
            let mut travel_sum = 0.0;

            let mut count = 0;

            // Sum up all of our wheel values to determine average forward wheel travel and average local
            // x-axis displacement.
            for (travel, _) in initial_forward_wheel_data.iter().flatten() {
                travel_sum += travel;
                count += 1;
            }

            if count != 0 {
                travel_sum / f64::from(count)
            } else {
                0.0
            }
        };

        let data = Rc::new(RefCell::new(TrackingData {
            position: origin.into(),
            heading_offset: heading,
            raw_heading: initial_raw_heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(
                forward_wheels,
                sideways_wheels,
                gyro,
                data,
                parallel_forward_indicies,
                initial_forward_wheel_data,
                initial_sideways_wheel_data,
                initial_raw_heading,
                initial_forward_travel,
            )),
        }
    }

    /// Creates a new wheeled tracking system with no sideways tracking wheels.
    pub fn forward_only<T: RotarySensor + 'static, G: Gyro + 'static, const NUM_FORWARD: usize>(
        origin: impl Into<Vec2<f64>>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        gyro: Option<G>,
    ) -> Self {
        Self::new(
            origin,
            heading,
            forward_wheels,
            [] as [TrackingWheel<T>; 0],
            gyro,
        )
    }

    // MARK: Heading Calculation

    /// Determines the orientation of the robot.
    ///
    /// "raw" in this case refers to the fact that the angle returned by this method has not been offset by any amount
    /// (meaning the user's "initial heading" configuration isn't considered), and has unspecified bounds (it may be
    /// out of the range of [0, 2π]). The angle is guaranteed to be counterclockwise-positive, but is otherwise a raw
    /// reading from whatever sensor is being used to determine orientation (either tracking wheels or a [`Gyro`]).
    ///
    /// To determine the final heading value returned by [`Self::heading`], you must add the heading offset value and
    /// wrap the angle from [0, 2π] using [`Angle::wrapped_positive`].
    ///
    /// # Errors
    ///
    /// There are two ways to determine robot orientation with wheeled tracking. You can either use a [`Gyro`], or you can
    /// use two parallel tracking wheels with roughly the same offset (spacing) from the center of rotation on the robot.
    ///
    /// - If the gyro fails to return a value, then [`HeadingError::Imu`] will be returned, and a fallback wheeled heading
    ///   may be available to use in this error type if the tracking setup has parallel tracking wheels that support it.
    /// - If a tracking wheel fails then [`HeadingError::RotarySensor`] will be returned containing the underlying error
    ///   that occurred.
    ///
    /// # Panics
    ///
    /// An assertion will panic if both `gyro` and `parallel_wheels` is `None`. This should never happen.
    fn compute_raw_heading<G: Gyro, T: RotarySensor>(
        gyro: Option<&G>,
        parallel_wheels: Option<(&TrackingWheel<T>, &TrackingWheel<T>)>,
    ) -> Result<Angle, HeadingError<T>> {
        assert!(
            gyro.is_some() || parallel_wheels.is_some(),
            "No gyro or wheeled tracking sensors provided to compute_heading."
        );

        // Try to get a reading of the robot's heading from the gyro. We should only do this if the gyro
        // hasn't returned a port-related error before (flagged by the `gyro_invalid` variable). If it
        // has, the gyro has no chance of recovery and we should fallback to wheeled heading calculation.
        let gyro_rotation = gyro.as_ref().map(|gyro| gyro.heading());

        // Compute the unbounded robot orientation in radians. In the case of the gyro, this actually is bounded to [0, TAU]
        // already due to how gyro heading works, but this will be wrapped to [0, TAU] regardless later either way.
        let raw_heading = if let Some(Ok(gyro_heading)) = gyro_rotation {
            // gyros' frame of reference is NED (Z-Down), meaning heading increases as the robot turns
            // clockwise. We don't want this, since it doesn't match up with how the unit circle works
            // with cartesian coordinates (what we localize in), so we need to convert to a CCW+ angle
            // system.
            TAU - gyro_heading.as_radians()
        } else if let Some((left_wheel, right_wheel)) = parallel_wheels {
            // Distance between the left and right wheels.
            let track_width = left_wheel.offset.abs() + right_wheel.offset;

            // Nothing we can use if either of these disconnects, so all we can do is wait for them to
            // reconnect. Seriously, fix your wiring!
            let left_travel = left_wheel
                .travel()
                .map_err(|err| HeadingError::RotarySensor(err))?;
            let right_travel = right_wheel
                .travel()
                .map_err(|err| HeadingError::RotarySensor(err))?;

            (right_travel - left_travel) / track_width
        } else if let Some(Err(_)) = gyro_rotation {
            // gyro failed and we have no viable sensors to determine heading. Nothing we can do to recover from this.
            return Err(HeadingError::Imu(None));
        } else {
            unreachable!() // handled by the assertion at the top of this function
        }
        .rad();

        match gyro_rotation {
            // gyro failed but we have a wheeled heading source to fall back to.
            Some(Err(_)) => Err(HeadingError::Imu(Some(raw_heading))),
            _ => Ok(raw_heading),
        }
    }

    // MARK: Task

    #[allow(clippy::too_many_arguments)]
    async fn task<
        T: RotarySensor,
        U: RotarySensor,
        G: Gyro,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        mut gyro: Option<G>,
        data: Rc<RefCell<TrackingData>>,
        parallel_forward_indicies: Option<(usize, usize)>,
        mut prev_forward_wheel_data: [Result<(f64, f64), <T as RotarySensor>::Error>; NUM_FORWARD],
        mut prev_sideways_wheel_data: [Result<(f64, f64), <U as RotarySensor>::Error>;
            NUM_SIDEWAYS],
        mut prev_raw_heading: Angle,
        mut prev_forward_travel: f64,
    ) {
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let mut data = data.borrow_mut();

            let forward_wheel_data = forward_wheels
                .each_ref()
                .map(|wheel| wheel.travel().map(|travel| (travel, wheel.offset)));
            let sideways_wheel_data = sideways_wheels
                .each_ref()
                .map(|wheel| wheel.travel().map(|travel| (travel, wheel.offset)));

            // Calculate absolute robot orientation (heading).
            //
            // This can be done in two possible ways - Either using a gyro (if it is available and actually
            // working) or through the use of two parallel forward trackers. The former is generally far more
            // reliable and isn't prone to wheel slip.
            data.raw_heading = match Self::compute_raw_heading(
                gyro.as_ref(),
                parallel_forward_indicies.map(|(left_index, right_index)| {
                    (&forward_wheels[left_index], &forward_wheels[right_index])
                }),
            ) {
                // Cool
                Ok(raw_heading) => raw_heading,

                // We got an error from the gyro, which means it likely disconnected. Once a gyro disconnects it
                // will reclibrate upon regaining power, which will mess tracking up badly, so we need to stop using
                // it in this case and switched to a wheeled method of determining heading.
                Err(HeadingError::Imu(raw_wheel_heading)) => {
                    gyro = None; // Set gyro to `None` so we don't use it in the future.

                    // Use the backup wheeled heading value in the gyro failed.
                    if let Some(raw_wheel_heading) = raw_wheel_heading {
                        raw_wheel_heading
                    } else {
                        // If no backup heading is available, that means we have no means of determining heading
                        // in the future (because we have no parallel forward trackers), meaning there's no point
                        // in continuing this task. Womp womp.
                        return;
                    }
                }

                // Occurs if both the gyro failed and the backup heading source failed.
                Err(HeadingError::RotarySensor(_)) if gyro.is_some() => {
                    gyro = None; // No more gyro :(
                    continue;
                }

                // One of the tracking wheels failed and we don't have a gyro, so just wait for it to reconnect I guess.
                _ => continue,
            };

            // Change in raw heading from the previous loop iteration.
            let delta_heading = (data.raw_heading - prev_raw_heading).wrapped();

            // Average between the current and previous heading reading used conversion between
            // global and local coordinate displacements.
            //
            // No need to wrap since we only plug this into trig functions.
            let avg_heading = ((data.raw_heading + prev_raw_heading) / 2.0) + data.heading_offset;
            prev_raw_heading = data.raw_heading;

            let mut local_displacement: Vec2<f64> = Vec2::default();
            let unit_chord = 2.0 * (delta_heading / 2.0).sin();

            // MARK: Sideways Wheels

            // Doing all the stuff with sideways trackers in this block below.
            {
                let mut local_y_sum = 0.0;
                let mut count = 0;

                for (data, prev_data) in sideways_wheel_data.iter().zip(&prev_sideways_wheel_data) {
                    if let Ok((travel, _)) = data {
                        if let Ok((prev_travel, offset)) = prev_data {
                            let delta_travel = travel - prev_travel;
                            count += 1;

                            local_y_sum += if delta_heading == Angle::ZERO {
                                delta_travel
                            } else {
                                unit_chord * (delta_travel / delta_heading.as_radians() + offset)
                            };
                        }
                    }
                }

                if count != 0 {
                    local_displacement.y = local_y_sum / f64::from(count);
                }

                prev_sideways_wheel_data = sideways_wheel_data;
            }

            // MARK: Forward Wheels

            // Doing all the stuff with forward trackers in this block below.
            {
                let mut local_x_sum = 0.0;
                let mut travel_sum = 0.0;

                let mut count = 0;
                let mut prev_count = 0;

                // Sum up all of our wheel values to determine average forward wheel travel and average local
                // x-axis displacement.
                for (data, prev_data) in forward_wheel_data.iter().zip(&prev_forward_wheel_data) {
                    if let Ok((travel, _)) = data {
                        travel_sum += travel;
                        count += 1;

                        // For x-axis displacement, we need to calculate a delta of how much our wheel travel
                        // has changed. To do this, we need to have a record of both our current and previous
                        // wheel travel, meaning we can only consider wheels that returned `Ok(_)` in both the
                        // current AND previous loop iteration here.
                        if let Ok((prev_travel, offset)) = prev_data {
                            let delta_travel = travel - prev_travel;
                            prev_count += 1;

                            // NOTE: I get the feeling that this could be more efficient, since we already know
                            // if `delta_heading` is zero before the for-loop here, meaning it's kinda wasteful
                            // to check it each iteration. On the other hand, it probably make this code more
                            // cancerous than it already is...
                            local_x_sum += if delta_heading == Angle::ZERO {
                                delta_travel
                            } else {
                                // shoutout to my man nick btw
                                unit_chord * (delta_travel / delta_heading.as_radians() + offset)
                            };
                        }
                    }
                }

                // Average local x-axis displacement (change in forward positioning along the robot's reference
                // frame) using all functioning forward tracking wheels.
                if prev_count != 0 {
                    local_displacement.x = local_x_sum / f64::from(prev_count);
                }

                // Calculate the forward wheel travel.
                //
                // This calculates the average of all forward tracking sensors on the robot as an estimate of
                // "how far we've driven". It's used for basic straight driving motion.
                if count != 0 {
                    data.forward_travel = travel_sum / f64::from(count);
                }

                prev_forward_wheel_data = forward_wheel_data;
            };

            // Used for estimating instantaneous velocity from wheel deltas.
            let dt = prev_time.elapsed();
            prev_time = Instant::now();

            // Linear/angular drivetrain velocity estimation
            //
            // TODO: Any kind of "dx/dt"-style differentiations here are flawed and will return zero
            //       sometimes due to the sample rate of our loop being higher than the sample rate
            //       of our sensor. We should also maybe consider EMA filtering this or something.
            data.linear_velocity = (data.forward_travel - prev_forward_travel) / dt.as_secs_f64();
            prev_forward_travel = data.forward_travel;

            data.angular_velocity = gyro
                .as_ref()
                .and_then(|gyro| gyro.heading().ok())
                .map_or(delta_heading.as_radians() / dt.as_secs_f64(), |gyro_rate| {
                    gyro_rate.as_radians()
                });

            // Update global position by converting our local displacement vector into a global offset (by
            // rotating our local offset by our heading). Each iteration, we apply this estimate of our change
            // in position to get a new estimate of the global position.
            //
            // If all this seems like gibberish to you, check out <https://www.youtube.com/watch?v=ZW7T6EFyYnc>.
            data.position += local_displacement.rotated(avg_heading.as_radians());
        }
    }

    // MARK: Setters

    /// Offsets the currently tracked heading to a given [`Angle`].
    pub fn set_heading(&mut self, heading: Angle) {
        let mut data = self.data.borrow_mut();
        data.heading_offset = heading - data.raw_heading;
    }

    /// Sets the currently tracked position to a new point.
    pub fn set_position(&mut self, position: Vec2<f64>) {
        self.data.borrow_mut().position = position;
    }
}

impl TracksPosition for WheeledTracking {
    fn position(&self) -> Vec2<f64> {
        self.data.borrow().position
    }
}

impl TracksHeading for WheeledTracking {
    fn heading(&self) -> Angle {
        let data = self.data.borrow();

        // Apply heading offset and wrap from [0, 2π]
        (data.raw_heading + data.heading_offset).wrapped_positive()
    }
}

impl TracksForwardTravel for WheeledTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}

impl TracksVelocity for WheeledTracking {
    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }

    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }
}
