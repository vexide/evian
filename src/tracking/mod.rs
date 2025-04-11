//! Robot Localization and Tracking
//!
//! This module provides traits and types for implementing robot tracking systems
//! that estimate a robot's position and general state of motion over time. A tracking
//! system typically fuses data from multiple sensors (encoders, IMUs, vision, etc.) to
//! maintain an estimate of the robot's:
//!
//! - Position in 2D space
//! - Heading (orientation)
//! - Linear and angular velocities
//! - Distance traveled
//!
//! Several basic implementations of tracking are provided by this module as a reference, with
//! the ability to implement your own custom tracking setups using the [`Tracking`] trait.

pub mod sensor;
pub mod wheeled;

use crate::math::{Angle, Vec2};

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
