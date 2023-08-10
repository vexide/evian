use core::f64::consts::{FRAC_2_PI, PI};
use core::prelude::rust_2021::*;

use crate::{
    math::Vec2,
    sensors::{Gyro, RotarySensor},
};

/// A system that tracks the absolute position and heading of a mobile robot.
pub trait Tracking: Send + Sync {
    fn forward_travel(&self) -> f64;

    fn heading(&self) -> f64;
    fn set_heading(&mut self, heading: f64);

    fn position(&self) -> Vec2;
    fn set_position(&mut self, position: Vec2);

    fn update(&mut self);
}

/// A struct representing a wheel attached to a rotary sensor.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackingWheel<'a, T: RotarySensor> {
    pub sensor: &'a T,
    pub wheel_diameter: f64,
    pub gearing: Option<f64>,
}

impl<'a, T: RotarySensor> TrackingWheel<'a, T> {
    pub fn new(sensor: &'a T, wheel_diameter: f64, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            wheel_diameter,
            gearing,
        }
    }
}

impl<'a, T: RotarySensor> TrackingWheel<'a, T> {
    fn travel(&self) -> f64 {
        let wheel_circumference = self.wheel_diameter * PI;

        return (self.sensor.rotation() / FRAC_2_PI)
            * self.gearing.unwrap_or(1.0 / 1.0)
            * wheel_circumference;
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum HeadingMethod<T: Gyro> {
    Gyro(T),
    TrackWidth(f64),
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParallelWheelTracking<'a, 'b, T: RotarySensor, U: RotarySensor, V: Gyro> {
    position: Vec2,
    left_wheel: TrackingWheel<'a, T>,
    right_wheel: TrackingWheel<'b, U>,
    heading_method: HeadingMethod<V>,
    heading_offset: f64,
    prev_forward_travel: f64,
}

impl<'a, 'b, T: RotarySensor, U: RotarySensor, V: Gyro> ParallelWheelTracking<'a, 'b, T, U, V> {
    pub fn new(
        origin: Vec2,
        heading: f64,
        left_wheel: TrackingWheel<'a, T>,
        right_wheel: TrackingWheel<'b, U>,
        heading_method: HeadingMethod<V>,
    ) -> Self {
        Self {
            position: origin,
            left_wheel,
            right_wheel,
            heading_method,
            heading_offset: heading,
            prev_forward_travel: 0.0,
        }
    }
}

impl<'a, 'b, T: RotarySensor, U: RotarySensor, V: Gyro> Tracking
    for ParallelWheelTracking<'a, 'b, T, U, V>
{
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
            + match &self.heading_method {
                HeadingMethod::Gyro(gyro) => FRAC_2_PI - gyro.heading(),
                HeadingMethod::TrackWidth(track_width) => {
                    self.right_wheel.travel() - self.left_wheel.travel() / track_width
                }
            } % FRAC_2_PI
    }

    fn set_heading(&mut self, _heading: f64) {
        todo!();
    }

    fn update(&mut self) {
        let forward_travel = self.forward_travel();
        let delta_forward_travel = forward_travel - self.prev_forward_travel;

        // Find a position delta.
        // This is a vector relative to the previous position, and can be found by creating a vector with our
        // average forward travel as the y-axis, then rotating the y-axis about our current heading. This gives
        // a rough estimate of the change in position, but does not account for sideways motion.
        self.position += Vec2::new(0.0, delta_forward_travel).rotate(self.heading());
        self.prev_forward_travel = forward_travel;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default, Copy, Clone, PartialEq)]
    struct FakeSensor {
        rotation: f64,
    }

    impl RotarySensor for FakeSensor {
        fn rotation(&self) -> f64 {
            self.rotation
        }
        fn set_rotation(&self, _: f64) {}
        fn reset_rotation(&self) {}
    }

    #[test]
    fn make_tracking_wheel() {
        TrackingWheel {
            sensor: &FakeSensor::default(),
            wheel_diameter: 3.25,
            gearing: Some(1.0),
        };
    }

    #[test]
    fn track_parallel_wheel_movement() {
        let sensor = FakeSensor::default();

        let left_wheel = TrackingWheel::new(left_sensor, 3.25, Some(1.0));
        let right_wheel = left_wheel.clone();

        let tracking = ParallelWheelTracking::new(
            Vec2::default(),
            90.0,
            left_wheel,
            right_wheel,
            HeadingMethod::TrackWidth(10.0),
        );
    }
}
