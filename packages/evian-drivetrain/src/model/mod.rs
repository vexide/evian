//! Drivetrain models.
//!
//! This module provides types for modeling a robot's motion capabilities
//! through various drivetrain configurations.

use evian_math::{desaturate, Vec2};

mod differential;
mod mecanum;

pub use differential::Differential;
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
        let [left, right] = desaturate([throttle + steer, throttle - steer], 1.0);

        self.drive_tank(left, right)
    }
}
