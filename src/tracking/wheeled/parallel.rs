use alloc::rc::Rc;
use core::{cell::RefCell, f64::consts::TAU, time::Duration};
use vexide::{
    core::time::Instant,
    devices::smart::InertialSensor,
    prelude::{sleep, spawn, Task},
};

use crate::{
    math::{Angle, Vec2},
    prelude::TracksVelocity,
    tracking::{sensor::RotarySensor, TracksForwardTravel, TracksHeading, TracksPosition},
};

use super::{TrackingData, TrackingWheel};

#[derive(Debug)]
pub struct ParallelWheelTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl ParallelWheelTracking {
    pub fn new<T: RotarySensor + 'static, U: RotarySensor + 'static>(
        origin: Vec2<f64>,
        heading: Angle,
        left_wheel: TrackingWheel<T>,
        right_wheel: TrackingWheel<U>,
        imu: Option<InertialSensor>,
    ) -> Self {
        let data = Rc::new(RefCell::new(TrackingData {
            position: origin,
            heading,
            heading_offset: heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(left_wheel, right_wheel, imu, data)),
        }
    }

    fn pre_offset_heading<T: RotarySensor, U: RotarySensor>(
        left_wheel: &TrackingWheel<T>,
        right_wheel: &TrackingWheel<U>,
        imu: Option<&InertialSensor>,
        initial_raw_heading: Angle,
    ) -> Angle {
        let track_width = left_wheel.offset + right_wheel.offset;
        Angle::from_radians(if let Some(imu) = imu {
            if let Ok(heading) = imu.heading() {
                TAU - heading.to_radians()
            } else {
                (right_wheel.travel() - left_wheel.travel()) / track_width
            }
        } else {
            (right_wheel.travel() - left_wheel.travel()) / track_width
        }) - initial_raw_heading
    }

    async fn task<T: RotarySensor, U: RotarySensor>(
        left_wheel: TrackingWheel<T>,
        right_wheel: TrackingWheel<U>,
        imu: Option<InertialSensor>,
        data: Rc<RefCell<TrackingData>>,
    ) {
        let track_width = left_wheel.offset + right_wheel.offset;
        let initial_raw_heading =
            Self::pre_offset_heading(&left_wheel, &right_wheel, imu.as_ref(), Angle::ZERO);

        let mut prev_left_travel = 0.0;
        let mut prev_right_travel = 0.0;

        let mut prev_heading = Angle::ZERO;
        let mut prev_time = Instant::now();

        loop {
            sleep(Duration::from_millis(5)).await;
            let dt = prev_time.elapsed();

            let left_travel = left_wheel.travel();
            let right_travel = right_wheel.travel();
            let forward_travel = (left_wheel.travel() + right_wheel.travel()) / 2.0;

            let heading_offset = data.borrow().heading_offset;
            let heading = Self::pre_offset_heading(
                &left_wheel,
                &right_wheel,
                imu.as_ref(),
                initial_raw_heading,
            ) + heading_offset;

            let delta_left_travel = left_travel - prev_left_travel;
            let delta_right_travel = right_travel - prev_right_travel;
            let delta_forward_travel = (delta_left_travel + delta_right_travel) / 2.0;
            let delta_heading = heading - prev_heading;
            let avg_heading = prev_heading + (delta_heading / 2.0);

            let displacement = if delta_heading == Angle::ZERO {
                Vec2::from_polar(delta_forward_travel, avg_heading.as_radians())
            } else {
                Vec2::from_polar(
                    2.0 * (delta_forward_travel / delta_heading.as_radians())
                        * (delta_heading / 2.0).sin(),
                    avg_heading.as_radians(),
                )
            };

            data.replace_with(|prev_data| TrackingData {
                position: prev_data.position + displacement,
                heading,
                forward_travel,
                heading_offset,
                linear_velocity: delta_forward_travel / dt.as_secs_f64(),
                angular_velocity: if let Some(imu) = imu.as_ref() {
                    if let Ok(gyro_rate) = imu.gyro_rate() {
                        gyro_rate.z.to_radians()
                    } else {
                        0.0
                    }
                } else {
                    (delta_right_travel - delta_left_travel) / (track_width * dt.as_secs_f64())
                },
            });

            prev_left_travel = left_travel;
            prev_right_travel = right_travel;
            prev_heading = heading;
            prev_time = Instant::now();
        }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.data.borrow_mut().heading_offset = heading;
    }

    pub fn set_position(&mut self, position: impl Into<Vec2<f64>>) {
        self.data.borrow_mut().position = position.into();
    }
}

impl TracksPosition for ParallelWheelTracking {
    fn position(&self) -> Vec2<f64> {
        self.data.borrow().position
    }
}

impl TracksHeading for ParallelWheelTracking {
    fn heading(&self) -> Angle {
        self.data.borrow().heading
    }
}

impl TracksForwardTravel for ParallelWheelTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}

impl TracksVelocity for ParallelWheelTracking {
    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }

    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }
}
