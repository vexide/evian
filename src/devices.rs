use alloc::{vec::Vec, sync::Arc};
use vex_rt::{
    adi::{AdiEncoder, AdiGyro},
    imu::InertialSensor,
    motor::Motor,
    rotation::RotationSensor,
    rtos::Mutex,
};

pub type MotorGroup = Arc<Mutex<Vec<Motor>>>;

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor: Send + Sync + 'static {
    /// Get the device's measured rotation in radians.
    fn rotation(&self) -> f64;
}

impl RotarySensor for Arc<Mutex<Motor>> {
    fn rotation(&self) -> f64 {
        let motor = self.lock();
        motor.get_position().unwrap().to_radians()
    }
}

impl RotarySensor for MotorGroup {
    fn rotation(&self) -> f64 {
        let group = self.lock();

        let mut sum = 0.0;
        for motor in group.iter() {
            sum += motor.get_position().unwrap().to_radians();
        }

        f64::from(sum) / (group.len() as f64)
    }
}

impl RotarySensor for AdiEncoder {
    fn rotation(&self) -> f64 {
        (self.get().unwrap() as f64).to_radians()
    }
}

impl RotarySensor for RotationSensor {
    fn rotation(&self) -> f64 {
        (self.get_position().unwrap() as f64 / 100.0).to_radians()
    }
}

/// A sensor that can measure absolute orientation, for example a gyroscope or IMU
pub trait Gyro: Send + Sync + 'static {
    fn heading(&self) -> f64;
    fn reset_heading(&mut self);
}

impl Gyro for InertialSensor {
    fn heading(&self) -> f64 {
        self.get_heading().unwrap().to_radians()
    }

    fn reset_heading(&mut self) {
        self.reset().unwrap();
    }
}

impl Gyro for AdiGyro {
    fn heading(&self) -> f64 {
        self.get().unwrap().to_radians()
    }

    fn reset_heading(&mut self) {
        self.reset().unwrap();
    }
}
