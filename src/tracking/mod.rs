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

pub mod parallel_wheel;
pub mod sensor;
pub mod wheel;

use core::fmt::Debug;
use core::prelude::rust_2021::*;

use crate::math::Vec2;

/// A system that performs localization and returns telemetry on a mobile robot.
pub trait Tracking {
    /// Updates the tracking system and returns the latest state estimate.
    ///
    /// This method should be called periodically to maintain an up-to-date
    /// estimate of the robot's position and motion state.
    fn update(&mut self) -> TrackingContext;
}

/// State information about a robot's position and motion.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrackingContext {
    /// The estimated 2D position of the robot in its coordinate frame.
    pub position: Vec2,

    /// The robot's heading angle in radians.
    ///
    /// Positive angles represent counterclockwise rotation from the positive x-axis.
    pub heading: f64,

    /// The total distance the robot has traveled forward.
    ///
    /// This accumulates the robot's forward motion over time, regardless of
    /// turns or direction changes.
    pub forward_travel: f64,

    /// The robot's current forward velocity in distance units per second.
    ///
    /// Positive values indicate forward motion, negative values indicate
    /// backward motion.
    pub linear_velocity: f64,

    /// The robot's current rate of rotation in radians per second.
    ///
    /// Positive values typically indicate counterclockwise rotation,
    /// negative values indicate clockwise rotation.
    pub angular_velocity: f64,
}
