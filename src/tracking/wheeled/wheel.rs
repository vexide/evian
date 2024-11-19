//! Tracking Wheels

use core::f64::consts::PI;

use crate::tracking::sensor::RotarySensor;

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
