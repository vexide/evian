//! Differential Drivetrain Commands
//! 
//! This module provides implementations of various motion algorithms for differential
//! drivetrains. Each algorithm implements the [`Command`] trait to provide voltage
//! updates to a drivetrain until it has settled.

pub mod basic;
pub mod point;
