use core::f64::consts::FRAC_2_PI;
use alloc::{vec::Vec, sync::Arc};
use vex_rt::{
    adi::{AdiEncoder, AdiGyro},
    imu::InertialSensor,
    motor::Motor,
    rotation::RotationSensor,
    rtos::Mutex,
};

pub type ThreadsafeMotor = Arc<Mutex<Motor>>;
pub type ThreadsafeMotorGroup = Arc<Mutex<Vec<Motor>>>;

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor: Send + Sync + 'static {
    /// Get the device's measured rotation in radians.
    fn rotation(&self) -> Result<f64, RotarySensorError>;
}

#[derive(Debug, PartialEq, Eq)]
pub struct RotarySensorError;

impl RotarySensor for Arc<Mutex<Motor>> {
    fn rotation(&self) -> Result<f64, RotarySensorError> {
        let motor = self.lock();

        Ok(motor.get_position().map_err(|_| RotarySensorError)?.to_radians())
    }
}

impl RotarySensor for ThreadsafeMotorGroup {
    fn rotation(&self) -> Result<f64, RotarySensorError> {
        let group = self.lock();

        let mut sum = 0.0;
        for motor in group.iter() {
            sum += motor.get_position().map_err(|_| RotarySensorError)?.to_radians();
        }

        Ok(sum / (group.len() as f64))
    }
}

impl RotarySensor for AdiEncoder {
    fn rotation(&self) -> Result<f64, RotarySensorError> {
        
        Ok((self.get().map_err(|_| RotarySensorError)? as f64).to_radians())
    }
}

impl RotarySensor for RotationSensor {
    fn rotation(&self) -> Result<f64, RotarySensorError> {
        Ok((self.get_position().map_err(|_| RotarySensorError)? as f64 / 100.0).to_radians())
    }
}

/// A sensor that can measure absolute orientation, for example a gyroscope or IMU
pub trait Gyro: Send + Sync + 'static {
    fn heading(&self) -> Result<f64, GyroError>;
}

#[derive(Debug, PartialEq, Eq)]
pub struct GyroError;

impl Gyro for InertialSensor {
    fn heading(&self) -> Result<f64, GyroError> {
        Ok(FRAC_2_PI - self.get_heading().map_err(|_| GyroError)?.to_radians())
    }
}

impl Gyro for AdiGyro {
    fn heading(&self) -> Result<f64, GyroError> {
        Ok(FRAC_2_PI - self.get().map_err(|_| GyroError)?.to_radians())
    }
}
