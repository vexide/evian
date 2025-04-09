use crate::math::{Angle, Vec2};

use alloc::rc::Rc;
use core::{
    cell::RefCell,
    f64::consts::{PI, TAU},
};
use vexide::{
    devices::smart::{InertialSensor, Motor},
    prelude::{sleep, spawn, RotationSensor, Task},
};

use crate::tracking::sensor::RotarySensor;
use crate::tracking::{TracksForwardTravel, TracksHeading, TracksPosition};

use super::TracksVelocity;

/// A wheel attached to a rotary sensor for position tracking.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackingWheel<T: RotarySensor> {
    pub sensor: T,
    pub wheel_diameter: f64,
    pub offset: f64,
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
    /// # Error Handling
    ///
    /// If sensor reading fails, this function returns `0.0`.
    pub fn travel(&self) -> f64 {
        let wheel_circumference = self.wheel_diameter * PI;

        self.sensor.position().unwrap_or_default().as_revolutions()
            * self.gearing.unwrap_or(1.0)
            * wheel_circumference
    }
}

/// Generic tracking data returned by [`ParallelWheelTracking`] and [`PerpendicularWheelTracking`].
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    position: Vec2<f64>,
    heading: Angle,
    heading_offset: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

#[derive(Debug)]
pub struct WheeledTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl WheeledTracking {
    pub fn new<
        T: RotarySensor + 'static,
        U: RotarySensor + 'static,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        origin: Vec2<f64>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        imu: Option<InertialSensor>,
    ) -> Self {
        const {
            assert!(
                NUM_FORWARD > 0,
                "Wheeled tracking requires at least one forward tracking wheel."
            );
        }
        
        assert!(
            NUM_FORWARD >= 2 || imu.is_some(),
            "Wheeled tracking requires an IMU or at least two parallel forward tracking wheels to determine robot orientation."
        );

        let data = Rc::new(RefCell::new(TrackingData {
            position: origin,
            heading_offset: heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(forward_wheels, sideways_wheels, imu, data)),
        }
    }

    pub fn forward_only<
        T: RotarySensor + 'static,
        const NUM_FORWARD: usize,
    >(
        origin: Vec2<f64>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        imu: Option<InertialSensor>,
    ) -> Self {
        Self::new(origin, heading, forward_wheels, [] as [TrackingWheel<T>; 0], imu)
    }

    fn pre_offset_heading(imu: &InertialSensor, initial_raw_heading: Angle) -> Angle {
        Angle::from_radians(if let Ok(heading) = imu.heading() {
            TAU - heading.to_radians()
        } else {
            0.0
        }) - initial_raw_heading
    }

    async fn task<
        T: RotarySensor,
        U: RotarySensor,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        imu: Option<InertialSensor>,
        data: Rc<RefCell<TrackingData>>,
    ) {
        todo!()
        // let mut prev_forward_travel = 0.0;
        // let mut prev_sideways_travel = 0.0;
        // let mut prev_heading = Angle::ZERO;

        // loop {
        //     let forward_travel = forward_wheels.map(|wheel| wheel.travel()).iter().sum() / F;
        //     let sideways_travel = sideways_wheels.map(|wheel| wheel.travel()).iter().sum() / S;
        //     let heading_offset = data.borrow().heading_offset;
        //     let heading =
        //         (TAU - imu.heading().unwrap_or_default().to_radians()).rad() + heading_offset;

        //     let delta_forward_travel = forward_travel - prev_forward_travel;
        //     let delta_sideways_travel = sideways_travel - prev_sideways_travel;
        //     let delta_heading = heading - prev_heading;

        //     let avg_heading = prev_heading + (delta_heading / 2.0);

        //     let displacement = if delta_heading == Angle::ZERO {
        //         Vec2::new(delta_forward_travel, delta_sideways_travel)
        //     } else {
        //         Vec2::new(
        //             2.0 * (delta_heading / 2.0).sin()
        //                 * (delta_sideways_travel / delta_heading.as_radians()
        //                     + sideways_wheel.offset),
        //             2.0 * (delta_heading / 2.0).sin()
        //                 * (delta_forward_travel / delta_heading.as_radians()
        //                     + forward_wheel.offset),
        //         )
        //     }
        //     .rotated(avg_heading.as_radians());

        //     data.replace_with(|prev_data| TrackingData {
        //         position: prev_data.position + displacement,
        //         heading,
        //         forward_travel,
        //         heading_offset,
        //         // TODO
        //         linear_velocity: 0.0,
        //         angular_velocity: 0.0,
        //     });

        //     prev_sideways_travel = sideways_travel;
        //     prev_forward_travel = forward_travel;
        //     prev_heading = heading;

        //     sleep(Motor::WRITE_INTERVAL).await;
        // }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.data.borrow_mut().heading_offset = heading - self.heading();
    }

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
        self.data.borrow().heading
    }
}

impl TracksForwardTravel for WheeledTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}

impl TracksVelocity for WheeledTracking {
    fn angular_velocity(&self) -> f64 {
        todo!()
    }

    fn linear_velocity(&self) -> f64 {
        todo!()
    }
}