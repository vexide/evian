//! Gyroscope trait

use evian_math::Angle;
use vexide::devices::smart::imu::{InertialError, InertialSensor};

/// A "gyroscope", or a sensor that measures the robot's heading and angular velocity
pub trait Gyro {
    /// The error returned when [`Gyro::heading`] or [`Gyro::angular_velocity`] fails. This
    /// describes a hardware error.
    type Error;

    /// Returns the heading of the robot as an [`Angle`]
    fn heading(&self) -> Result<Angle, Self::Error>;
    /// Returns the horizontal angular velocity
    fn angular_velocity(&self) -> Result<f64, Self::Error>;
}

impl Gyro for InertialSensor {
    type Error = InertialError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        InertialSensor::heading(self).map(Angle::from_radians)
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        InertialSensor::gyro_rate(self).map(|rate| rate.z)
    }
}
