use core::f64::consts::TAU;
use vexide::devices::smart::InertialSensor;

use crate::math::{Angle, Vec2};

use super::{sensor::RotarySensor, wheel::TrackingWheel, Tracking, TrackingData};

#[derive(Debug, PartialEq)]
pub struct ParallelWheelTracking<T: RotarySensor, U: RotarySensor> {
    position: Vec2,
    left_wheel: TrackingWheel<T>,
    right_wheel: TrackingWheel<U>,

    imu: Option<InertialSensor>,
    heading_offset: Angle,
    prev_forward_travel: f64,
    prev_heading: Angle,
}

impl<T: RotarySensor, U: RotarySensor> ParallelWheelTracking<T, U> {
    pub fn new(
        origin: Vec2,
        heading: Angle,
        left_wheel: TrackingWheel<T>,
        right_wheel: TrackingWheel<U>,
        gyro: Option<InertialSensor>,
    ) -> Self {
        Self {
            position: origin,
            left_wheel,
            right_wheel,
            imu: gyro,
            heading_offset: heading,
            prev_forward_travel: 0.0,
            prev_heading: Angle::default(),
        }
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.heading_offset = heading - self.heading();
    }

    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn track_width(&self) -> f64 {
        self.left_wheel.offset + self.right_wheel.offset
    }

    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn heading(&self) -> Angle {
        Angle::from_radians(
            (self.heading_offset.as_radians()
                + if let Some(imu) = &self.imu {
                    if let Ok(heading) = imu.heading() {
                        TAU - heading.to_radians()
                    } else {
                        (self.right_wheel.travel() - self.left_wheel.travel()) / self.track_width()
                    }
                } else {
                    (self.right_wheel.travel() - self.left_wheel.travel()) / self.track_width()
                })
                % TAU,
        )
    }

    pub fn forward_travel(&self) -> f64 {
        (self.left_wheel.travel() + self.right_wheel.travel()) / 2.0
    }
}

impl<T: RotarySensor, U: RotarySensor> Tracking for ParallelWheelTracking<T, U> {
    fn update(&mut self) -> TrackingData {
        let forward_travel = self.forward_travel();
        let heading = self.heading();

        let delta_forward_travel = forward_travel - self.prev_forward_travel;
        let delta_heading = heading - self.prev_heading;
        let avg_heading = self.prev_heading + (delta_heading / 2.0);

        if delta_heading.as_radians() == 0.0 {
            self.position += Vec2::from_polar(delta_forward_travel, avg_heading.as_radians());
        } else {
            self.position += Vec2::from_polar(
                2.0 * (delta_forward_travel / delta_heading.as_radians())
                    * (delta_heading / 2.0).sin(),
                avg_heading.as_radians(),
            );
        }

        self.prev_forward_travel = forward_travel;
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
