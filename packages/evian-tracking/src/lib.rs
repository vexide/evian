//! Robot localization and tracking.

#![no_std]

extern crate alloc;

pub mod sensor;
pub mod wheeled;

use evian_math::{Angle, Vec2};

pub trait TracksPosition {
    /// Return's the robot's position on a 2D cartesian coordinate plane measured
    /// in wheel units.
    fn position(&self) -> Vec2<f64>;
}

pub trait TracksHeading {
    /// Returns the robot's orientation bounded from [0, 2π] radians.
    fn heading(&self) -> Angle;
}

pub trait TracksVelocity {
    /// Returns the robot's estimated linear velocity in wheel units/sec.
    fn linear_velocity(&self) -> f64;

    /// Returns the robot's estimated angular velocity in radians/sec.
    fn angular_velocity(&self) -> f64;
}

pub trait TracksForwardTravel {
    /// Returns the average forward wheel travel of the robot in wheel units.
    fn forward_travel(&self) -> f64;
}
