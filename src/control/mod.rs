//! Control Theory Primitives
//!
//! This module provides basic building-blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

use core::time::Duration;

mod pid;
mod settler;

pub use pid::{AngularPid, Pid};
pub use settler::Settler;

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
