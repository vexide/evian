use core::f64::consts::TAU;
use vexide::{core::float::Float, devices::smart::InertialSensor};

use crate::math::Vec2;

use super::{sensor::RotarySensor, wheel::TrackingWheel, Tracking, TrackingData};

#[derive(Debug, PartialEq)]
pub struct ParallelWheelTracking<T: RotarySensor, U: RotarySensor> {
    forward_wheel: TrackingWheel<T>,
    sideways_wheel: TrackingWheel<U>,
    imu: InertialSensor,

    position: Vec2,
    heading_offset: f64,

    prev_forward_travel: f64,
    prev_sideways_travel: f64,
    prev_heading: f64,
}

impl<T: RotarySensor, U: RotarySensor> ParallelWheelTracking<T, U> {
    pub fn new(
        origin: Vec2,
        heading: f64,
        forward_wheel: TrackingWheel<T>,
        sideways_wheel: TrackingWheel<U>,
        gyro: InertialSensor,
    ) -> Self {
        Self {
            position: origin,
            forward_wheel,
            sideways_wheel,
            imu: gyro,
            heading_offset: heading,
            prev_forward_travel: 0.0,
            prev_sideways_travel: 0.0,
            prev_heading: 0.0,
        }
    }

    pub fn set_heading(&mut self, heading: f64) {
        self.heading_offset = heading - self.heading();
    }

    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn heading(&self) -> f64 {
        (TAU - self.imu.heading().unwrap_or_default().to_radians()) % TAU
    }

    pub fn forward_travel(&self) -> f64 {
        self.forward_wheel.travel()
    }

    pub fn sideways_travel(&self) -> f64 {
        self.sideways_wheel.travel()
    }
}

impl<T: RotarySensor, U: RotarySensor> Tracking for ParallelWheelTracking<T, U> {
    fn update(&mut self) -> TrackingData {
        let forward_travel = self.forward_travel();
        let sideways_travel = self.forward_travel();
        let heading = self.heading();

        let delta_forward_travel = forward_travel - self.prev_forward_travel;
        let delta_sideways_travel = sideways_travel - self.prev_sideways_travel;
        let delta_heading = heading - self.prev_heading;
        let avg_heading = self.prev_heading + (delta_heading / 2.0);

        if delta_heading == 0.0 {
            self.position +=
                Vec2::new(delta_forward_travel, delta_sideways_travel).rotated(avg_heading);
        } else {
            self.position += Vec2::new(
                2.0 * (delta_heading / 2.0).sin()
                    * (delta_sideways_travel / delta_heading + self.sideways_wheel.offset),
                2.0 * (delta_heading / 2.0).sin()
                    * (delta_forward_travel / delta_heading + self.forward_wheel.offset),
            )
            .rotated(avg_heading);
        }

        self.prev_forward_travel = forward_travel;
        self.prev_sideways_travel = sideways_travel;
        self.prev_heading = heading;

        TrackingData {
            position: self.position,
            heading,
            forward_travel,
            linear_velocity: Default::default(),
            angular_velocity: Default::default(),
        }
    }
}
