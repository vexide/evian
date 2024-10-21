use core::f64::consts::FRAC_2_PI;
use vexide::{core::float::Float, devices::smart::InertialSensor};

use crate::math::Vec2;

use super::{sensor::RotarySensor, wheel::TrackingWheel, Tracking, TrackingContext};

#[derive(Debug, PartialEq)]
pub struct ParallelWheelTracking<T: RotarySensor, U: RotarySensor> {
    position: Vec2,
    left_wheel: TrackingWheel<T>,
    right_wheel: TrackingWheel<U>,
    gyro: Option<InertialSensor>,
    heading_offset: f64,
    prev_forward_travel: f64,
    prev_heading: f64,
}

impl<T: RotarySensor, U: RotarySensor> ParallelWheelTracking<T, U> {
    pub fn new(
        origin: Vec2,
        heading: f64,
        left_wheel: TrackingWheel<T>,
        right_wheel: TrackingWheel<U>,
        gyro: Option<InertialSensor>,
    ) -> Self {
        Self {
            position: origin,
            left_wheel,
            right_wheel,
            gyro,
            heading_offset: heading,
            prev_forward_travel: 0.0,
            prev_heading: 0.0,
        }
    }
}

impl<T: RotarySensor, U: RotarySensor> ParallelWheelTracking<T, U> {
    pub fn set_heading(&mut self, heading: f64) {
        self.heading_offset = heading - self.heading();
    }
    
    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn track_width(&self) -> f64 {
        self.left_wheel.offset + self.right_wheel.offset
    }

    pub fn heading(&self) -> f64 {
        self.heading_offset
            + if let Some(gyro) = &self.gyro {
                if let Ok(heading) = gyro.heading() {
                    heading.to_radians()
                } else {
                    (self.right_wheel.travel() - self.left_wheel.travel()) / self.track_width()
                }
            } else {
                (self.right_wheel.travel() - self.left_wheel.travel()) / self.track_width()
            } % FRAC_2_PI
    }

    pub fn forward_travel(&self) -> f64 {
        (self.left_wheel.travel() + self.right_wheel.travel()) / 2.0
    }
}

impl<T: RotarySensor, U: RotarySensor> Tracking for ParallelWheelTracking<T, U> {
    fn update(&mut self) -> TrackingContext {
        let forward_travel = self.forward_travel();
        let heading = self.heading();

        let delta_forward_travel = forward_travel - self.prev_forward_travel;
        let delta_heading = heading - self.prev_heading;

        self.position += Vec2::from_polar(
            2.0 * (delta_forward_travel / delta_heading) * (heading / 2.0).sin(),
            (self.prev_heading + delta_heading) / 2.0,
        );
        self.prev_forward_travel = forward_travel;

        TrackingContext {
            position: self.position,
            heading,
            forward_travel,
            linear_velocity: todo!(),
            angular_velocity: todo!(),
        }
    }
}
