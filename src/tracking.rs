use core::f64::consts::{FRAC_2_PI, PI};
use core::fmt::Debug;
use core::prelude::rust_2021::*;
use num_traits::real::Real;
use vexide::devices::position::Position;
use vexide::devices::smart::InertialSensor;

use crate::{devices::RotarySensor, math::Vec2};

/// A system that performs localization and returns telemetry on a mobile robot.
pub trait Tracking: Send + 'static {
    fn forward_travel(&self) -> f64;

    fn heading(&self) -> f64;
    fn set_heading(&mut self, heading: f64);

    fn position(&self) -> Vec2;
    fn set_position(&mut self, position: Vec2);

    fn update(&mut self) -> TrackingContext;
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrackingContext {
    pub position: Vec2,
    pub heading: f64,
    pub forward_travel: f64,
}

/// A struct representing a wheel attached to a rotary sensor.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackingWheel<T: RotarySensor> {
    pub sensor: T,
    pub wheel_diameter: f64,
    pub offset: f64,
    pub gearing: Option<f64>,
}

impl<T: RotarySensor> TrackingWheel<T> {
    pub fn new(sensor: T, wheel_diameter: f64, offset: f64, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            wheel_diameter,
            offset,
            gearing,
        }
    }
}

impl<T: RotarySensor> TrackingWheel<T> {
    fn travel(&self) -> f64 {
        let wheel_circumference = self.wheel_diameter * PI;

        return self
            .sensor
            .position()
            .unwrap_or(Position::from_counts(0))
            .into_degrees()
            / 360.0
            * self.gearing.unwrap_or(1.0)
            * wheel_circumference;
    }
}

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
    fn track_width(&self) -> f64 {
        self.left_wheel.offset + self.right_wheel.offset
    }
}

impl<T: RotarySensor, U: RotarySensor> Tracking for ParallelWheelTracking<T, U> {
    fn position(&self) -> Vec2 {
        self.position
    }

    fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    fn forward_travel(&self) -> f64 {
        (self.left_wheel.travel() + self.right_wheel.travel()) / 2.0
    }

    fn heading(&self) -> f64 {
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

    fn set_heading(&mut self, heading: f64) {
        self.heading_offset = heading - self.heading();
    }

    fn update(&mut self) -> TrackingContext {
        let forward_travel = self.forward_travel();
        let heading = self.heading();

        let delta_forward_travel = forward_travel - self.prev_forward_travel;
        let delta_heading = heading - self.prev_heading;

        // Find a position delta.
        // This is a vector relative to the previous position, and can be found by creating a vector with our
        // average forward travel as the y-axis, then rotating the y-axis about our current heading. This gives
        // a rough estimate of the change in position, but does not account for sideways motion.
        self.position += Vec2::from_polar(
            2.0 * (delta_forward_travel / delta_heading) * (heading / 2.0).sin(),
            (self.prev_heading + delta_heading) / 2.0,
        );
        self.prev_forward_travel = forward_travel;

        TrackingContext {
            position: self.position(),
            heading: self.heading(),
            forward_travel: self.forward_travel(),
        }
    }
}
