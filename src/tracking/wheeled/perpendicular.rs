use alloc::rc::Rc;
use core::{cell::RefCell, f64::consts::TAU};
use vexide::{
    devices::smart::{InertialSensor, Motor},
    prelude::{sleep, spawn, Task},
};

use crate::{
    math::{Angle, IntoAngle, Vec2},
    prelude::TracksVelocity,
    tracking::{sensor::RotarySensor, TracksForwardTravel, TracksHeading, TracksPosition},
};

use super::{TrackingData, TrackingWheel};

#[derive(Debug)]
pub struct PerpendicularWheelTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl PerpendicularWheelTracking {
    pub fn new<T: RotarySensor + 'static, U: RotarySensor + 'static>(
        origin: Vec2<f64>,
        heading: Angle,
        forward_wheel: TrackingWheel<T>,
        sideways_wheel: TrackingWheel<U>,
        imu: InertialSensor,
    ) -> Self {
        let data = Rc::new(RefCell::new(TrackingData {
            position: origin,
            heading_offset: heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(forward_wheel, sideways_wheel, imu, data)),
        }
    }

    async fn task<T: RotarySensor, U: RotarySensor>(
        forward_wheel: TrackingWheel<T>,
        sideways_wheel: TrackingWheel<U>,
        imu: InertialSensor,
        data: Rc<RefCell<TrackingData>>,
    ) {
        let mut prev_forward_travel = 0.0;
        let mut prev_sideways_travel = 0.0;
        let mut prev_heading = Angle::ZERO;

        loop {
            let forward_travel = forward_wheel.travel();
            let sideways_travel = sideways_wheel.travel();
            let heading_offset = data.borrow().heading_offset;
            let heading =
                (TAU - imu.heading().unwrap_or_default().to_radians()).rad() + heading_offset;

            let delta_forward_travel = forward_travel - prev_forward_travel;
            let delta_sideways_travel = sideways_travel - prev_sideways_travel;
            let delta_heading = heading - prev_heading;

            let avg_heading = prev_heading + (delta_heading / 2.0);

            let displacement = if delta_heading == Angle::ZERO {
                Vec2::new(delta_forward_travel, delta_sideways_travel)
            } else {
                Vec2::new(
                    2.0 * (delta_heading / 2.0).sin()
                        * (delta_sideways_travel / delta_heading.as_radians()
                            + sideways_wheel.offset),
                    2.0 * (delta_heading / 2.0).sin()
                        * (delta_forward_travel / delta_heading.as_radians()
                            + forward_wheel.offset),
                )
            }
            .rotated(avg_heading.as_radians());

            data.replace_with(|prev_data| TrackingData {
                position: prev_data.position + displacement,
                heading,
                forward_travel,
                heading_offset,
                // TODO
                linear_velocity: 0.0,
                angular_velocity: 0.0,
            });

            prev_sideways_travel = sideways_travel;
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

impl TracksPosition for PerpendicularWheelTracking {
    fn position(&self) -> Vec2<f64> {
        self.data.borrow().position
    }
}

impl TracksHeading for PerpendicularWheelTracking {
    fn heading(&self) -> Angle {
        self.data.borrow().heading
    }
}

impl TracksForwardTravel for PerpendicularWheelTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}

impl TracksVelocity for PerpendicularWheelTracking {
    fn angular_velocity(&self) -> f64 {
        todo!("velocity tracking is not implemented for PerpendicularWheelTracking yet.")
    }

    fn linear_velocity(&self) -> f64 {
        todo!("velocity tracking is not implemented for PerpendicularWheelTracking yet.")
    }
}
