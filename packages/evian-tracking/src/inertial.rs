//! Provides proper multiple inertial support

use evian_math::Angle;
use vexide::{
    devices::{math::Vector3, smart::imu::InertialError},
    prelude::InertialSensor,
};

/// Generic IMU trait, allowing for use of anything that implements this as a method of getting heading
pub trait Gyro {
    /// Method of getting raw gyroscope readings
    fn g_gyro_rate(&self) -> Result<Vector3<Angle>, InertialError>;
    /// Method of retreiving heading
    fn g_heading(&mut self) -> Result<Angle, InertialError>;
    /// get raw angular acceleration readings
    fn g_angular_velocity(&mut self) -> Result<Angle, InertialError>;
}

impl Gyro for InertialSensor {
    fn g_angular_velocity(&mut self) -> Result<Angle, InertialError> {
        let gyro_rate = self.g_gyro_rate();
        match gyro_rate {
            Ok(s) => Ok(s.z),
            Err(e) => Err(e),
        }
    }
    fn g_gyro_rate(&self) -> Result<Vector3<Angle>, InertialError> {
        let gyro_rate = self.gyro_rate();
        match gyro_rate {
            Ok(s) => Ok(Vector3 {
                x: Angle::from_radians(s.x),
                y: Angle::from_radians(s.y),
                z: Angle::from_radians(s.z),
            }),
            Err(e) => Err(e),
        }
    }
    fn g_heading(&mut self) -> Result<Angle, InertialError> {
        let heading = self.heading();

        match heading {
            Ok(s) => Ok(Angle::from_radians(s)),
            Err(e) => Err(e),
        }
    }
}
