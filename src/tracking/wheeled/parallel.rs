use alloc::rc::Rc;
use core::{cell::RefCell, f64::consts::TAU};
use vexide::{
    devices::smart::{InertialSensor, Motor},
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
            heading_offset: heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(left_wheel, right_wheel, imu, data)),
        }
    }

    async fn task<T: RotarySensor, U: RotarySensor>(
        left_wheel: TrackingWheel<T>,
        right_wheel: TrackingWheel<U>,
        imu: Option<InertialSensor>,
        data: Rc<RefCell<TrackingData>>,
    ) {
        let mut prev_forward_travel = 0.0;
        let mut prev_heading = Angle::ZERO;

        loop {
            let forward_travel = (left_wheel.travel() + right_wheel.travel()) / 2.0;
            let heading_offset = data.borrow().heading_offset;
            let heading = {
                let track_width = left_wheel.offset + right_wheel.offset;
                let raw_heading = if let Some(ref imu) = imu {
                    if let Ok(heading) = imu.heading() {
                        TAU - heading.to_radians()
                    } else {
                        (right_wheel.travel() - left_wheel.travel()) / track_width
                    }
                } else {
                    (right_wheel.travel() - left_wheel.travel()) / track_width
                };

                Angle::from_radians((heading_offset.as_radians() + raw_heading) % TAU)
            };

            let delta_forward_travel = forward_travel - prev_forward_travel;
            let delta_heading = heading - prev_heading;
            let avg_heading = prev_heading + (delta_heading / 2.0);

            let displacement = if delta_heading.as_radians() == 0.0 {
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
            });

            prev_forward_travel = forward_travel;
            prev_heading = heading;

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.data.borrow_mut().heading_offset = heading - self.heading();
    }

    pub fn set_position(&mut self, position: Vec2<f64>) {
        self.data.borrow_mut().position = position;
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
        todo!()
    }

    fn linear_velocity(&self) -> f64 {
        todo!()
    }
}
