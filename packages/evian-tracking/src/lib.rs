//! Robot localization and tracking.

#![no_std]

extern crate alloc;

mod sensor;
pub mod wheeled;

pub use sensor::RotarySensor;

use evian_math::{Angle, Vec2};

/// A tracking system that localizes a robot's 2D position.
pub trait TracksPosition {
    /// Return's the robot's position on a 2D cartesian coordinate plane measured
    /// in wheel units.
    fn position(&self) -> Vec2<f64>;
}

/// A tracking system that tracks a robot's absolute orientation.
pub trait TracksHeading {
    /// Returns the robot's orientation bounded from [0, 2π] radians.
    fn heading(&self) -> Angle;
}

/// A tracking system that tracks a robot's linear and angular velocity.
pub trait TracksVelocity {
    /// Returns the robot's estimated linear velocity in wheel units/sec.
    fn linear_velocity(&self) -> f64;

    /// Returns the robot's estimated angular velocity in radians/sec.
    fn angular_velocity(&self) -> f64;
}

/// A tracking system that tracks a robot's forward wheel travel.
pub trait TracksForwardTravel {
    /// Returns the average forward wheel travel of the robot in wheel units.
    fn forward_travel(&self) -> f64;
}
