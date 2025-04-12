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

pub trait ControlLoop {
    type Input;
    type Output;

    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}
