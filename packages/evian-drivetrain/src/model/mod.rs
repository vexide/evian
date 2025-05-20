//! Drivetrain models.
//! 
//! This module provides types for modeling a robot's motion capabilities
//! through various drivetrain configurations.

use evian_math::Vec2;

mod differential;
mod h_drive;
mod kiwi;
mod mecanum;

pub use differential::Differential;
pub use h_drive::HDrive;
pub use kiwi::Kiwi;
pub use mecanum::Mecanum;

/// A collection of motors driving a mobile robot.
pub trait DrivetrainModel {
    type Error;
}

/// A drivetrain model that supports holonomic inverse kinematics.
pub trait Holonomic: DrivetrainModel {
    fn drive_vector(&mut self, vector: Vec2<f64>, turn: f64) -> Result<(), Self::Error>;
}

/// A drivetrain model that supports "arcade drive" (forward/turn) inverse kinematics.
pub trait Arcade: DrivetrainModel {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error>;
}

/// A drivetrain model that supports "tank drive" (left/right) inverse kinematics.
pub trait Tank: DrivetrainModel {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error>;
}

impl<T: Tank> Arcade for T {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let (left, right) = desaturate(throttle + steer, throttle - steer, 1.0);

        self.drive_tank(left, right)
    }
}

pub(crate) fn desaturate(mut a: f64, mut b: f64, max: f64) -> (f64, f64) {
    let larger_magnitude = a.abs().max(b.abs()) / max;

    if larger_magnitude > 1.0 {
        a /= larger_magnitude;
        b /= larger_magnitude;
    }

    (a, b)
}
