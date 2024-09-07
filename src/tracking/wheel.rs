use core::f64::consts::PI;

use super::sensor::RotarySensor;

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
    pub fn travel(&self) -> f64 {
        let wheel_circumference = self.wheel_diameter * PI;

        self.sensor.position().unwrap_or_default().as_revolutions()
            * self.gearing.unwrap_or(1.0)
            * wheel_circumference
    }
}
