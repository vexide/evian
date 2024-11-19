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
    fn position(&self) -> Vec2<f64>;
}

pub trait TracksHeading {
    fn heading(&self) -> Angle;
}

pub trait TracksVelocity {
    fn linear_velocity(&self) -> f64;
    fn angular_velocity(&self) -> f64;
}

pub trait TracksForwardTravel {
    fn forward_travel(&self) -> f64;
}
