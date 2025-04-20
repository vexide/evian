//! Control theory primitives.
//!
//! This module provides basic building blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

#![no_std]

pub mod loops;

mod tolerances;
pub use tolerances::Tolerances;
