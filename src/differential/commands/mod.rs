//! Differential Drivetrain Commands
//!
//! This module provides implementations of various motion algorithms for differential
//! drivetrains. Each algorithm implements the [`Command`] trait to provide voltage
//! updates to a drivetrain until it has settled.
//!
//! [`Command`]: crate::command::Command

pub mod basic;
pub mod mtp;