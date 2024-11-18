use core::f64::consts::TAU;
use vexide::devices::smart::InertialSensor;

use crate::math::{Angle, Vec2};

use super::{sensor::RotarySensor, wheel::TrackingWheel, Tracking, TrackingData};

#[derive(Debug, PartialEq)]
pub struct PerpendicularWheelTracking<T: RotarySensor, U: RotarySensor> {
    forward_wheel: TrackingWheel<T>,
    sideways_wheel: TrackingWheel<U>,
    imu: InertialSensor,

    position: Vec2,
    heading_offset: Angle,

    prev_forward_travel: f64,
    prev_sideways_travel: f64,
    prev_heading: Angle,
}

impl<T: RotarySensor, U: RotarySensor> PerpendicularWheelTracking<T, U> {
    pub fn new(
        origin: Vec2,
        heading: Angle,
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
            prev_heading: Angle::default(),
        }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.heading_offset = heading - self.heading();
    }

    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn heading(&self) -> Angle {
        Angle::from_radians((TAU - self.imu.heading().unwrap_or_default().to_radians()) % TAU)
    }

    pub fn forward_travel(&self) -> f64 {
        self.forward_wheel.travel()
    }

    pub fn sideways_travel(&self) -> f64 {
        self.sideways_wheel.travel()
    }
}

impl<T: RotarySensor, U: RotarySensor> Tracking for PerpendicularWheelTracking<T, U> {
    fn update(&mut self) -> TrackingData {
        let forward_travel = self.forward_travel();
        let sideways_travel = self.forward_travel();
        let heading = self.heading();

        let delta_forward_travel = forward_travel - self.prev_forward_travel;
        let delta_sideways_travel = sideways_travel - self.prev_sideways_travel;
        let delta_heading = heading - self.prev_heading;
        let avg_heading = self.prev_heading + (delta_heading / 2.0);

        if delta_heading.as_radians() == 0.0 {
            self.position += Vec2::new(delta_forward_travel, delta_sideways_travel)
                .rotated(avg_heading.as_radians());
        } else {
            self.position += Vec2::new(
                2.0 * (delta_heading / 2.0).sin()
                    * (delta_sideways_travel / delta_heading.as_radians()
                        + self.sideways_wheel.offset),
                2.0 * (delta_heading / 2.0).sin()
                    * (delta_forward_travel / delta_heading.as_radians()
                        + self.forward_wheel.offset),
            )
            .rotated(avg_heading.as_radians());
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
