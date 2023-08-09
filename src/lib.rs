#![feature(core_intrinsics)]
#![no_std]

extern crate alloc;

pub mod controller;
pub mod drivetrain;
pub mod math;
pub mod sensors;
pub mod tracking;

pub mod prelude {
    pub use crate::{controller::*, drivetrain::*, math::*, sensors::*, tracking::*};
}
