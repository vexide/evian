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
pub mod wheel;

pub mod parallel_wheel;
pub mod perpendicular_wheel;

use core::cell::RefCell;
use core::fmt::Debug;
use core::prelude::rust_2021::*;

use alloc::rc::Rc;

use crate::math::{Angle, Vec2};

/// A system that performs localization and returns telemetry on a mobile robot.
pub trait Tracking {
    /// Updates the tracking system and returns the latest state estimate.
    ///
    /// This method should be called periodically to maintain an up-to-date
    /// estimate of the robot's position and motion state.
    fn update(&mut self) -> TrackingData;
}

/// Blanket implementation for all `Rc<RefCell<T>>` wrappers of already implemented sensors.
impl<T: Tracking> Tracking for Rc<RefCell<T>> {
    fn update(&mut self) -> TrackingData {
        self.borrow_mut().update()
    }
}

/// State information about a robot's position and motion.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrackingData {
    /// The estimated 2D position of the robot in its coordinate frame.
    pub position: Vec2,

    /// The robot's heading angle in radians.
    ///
    /// Positive angles represent counterclockwise rotation from the positive x-axis.
    pub heading: Angle,

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
