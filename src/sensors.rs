use alloc::vec::Vec;
use vex_rt::{
    adi::{AdiEncoder, AdiGyro},
    imu::InertialSensor,
    motor::Motor,
    rotation::RotationSensor,
};

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor: Send + Sync {
    /// Get the device's measured rotation in radians.
    fn rotation(&self) -> f64;

    /// Set the current rotation angle as zero.
    fn reset_rotation(&mut self);
}

impl RotarySensor for Motor {
    fn rotation(&self) -> f64 {
        (self.get_position().unwrap()).to_radians()
    }

    fn reset_rotation(&mut self) {
        self.tare_position().unwrap();
    }
}

impl RotarySensor for Vec<Motor> {
    fn rotation(&self) -> f64 {
        let mut sum = 0.0;
        for motor in self.iter() {
            sum += motor.rotation();
        }

        f64::from(sum) / (self.len() as f64)
    }

    fn reset_rotation(&mut self) {
        for motor in self.iter_mut() {
            motor.reset_rotation();
        }
    }
}

impl RotarySensor for AdiEncoder {
    fn rotation(&self) -> f64 {
        (self.get().unwrap() as f64).to_radians()
    }

    fn reset_rotation(&mut self) {
        self.reset().unwrap();
    }
}

impl RotarySensor for RotationSensor {
    fn rotation(&self) -> f64 {
        (self.get_position().unwrap() as f64 / 100.0).to_radians()
    }

    fn reset_rotation(&mut self) {
        self.reset_position().unwrap();
    }
}

/// A sensor that can measure absolute orientation, for example a gyroscope or IMU
pub trait Gyro: Send + Sync {
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
