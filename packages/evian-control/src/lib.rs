//! Control theory primitives.
//!
//! This module provides basic building-blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

#![no_std]

extern crate alloc;

use core::time::Duration;

mod pid;
mod profile;
pub mod trajectory;

pub use pid::{AngularPid, Pid};
pub use profile::{TrapezoidalConstraints, TrapezoidalProfile};

/// A trait representing a generic control loop.
///
/// Control loops are fundamental to control systems, where a controller
/// continually adjusts its output to drive a control system to its desired value.
///
/// This trait defines the core behavior of such controllers, which takea `setpoint`
/// (the desired target value of the system) and optionally a `measurement` of the
/// system (recorded through a sensor or similar). The controller then computes an
/// output intended to minimize the system's *error*, which is the difference between
/// the `measurement` and the `setpoint`.
pub trait ControlLoop {
    /// The type of input measurements and setpoints that this controller takes.
    type Input;

    /// The type of output (control signal) this controller produces.
    type Output;

    /// Updates the control loop with the latest `measurement` and `setpoint` for the
    /// system, producing a corresponding control signal.
    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}
