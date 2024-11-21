//! Differential Drivetrain Commands
//!
//! This module provides implementations of various motion algorithms for differential
//! drivetrains. Each algorithm implements the [`Command`] trait to provide voltage
//! updates to a drivetrain until it has settled.
//!
//! [`Command`]: crate::command::Command

mod basic;
mod mtp;

mod settler;

pub use settler::Settler;
pub use basic::BasicMotion;
pub use mtp::PointToPoint;