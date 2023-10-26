#![feature(core_intrinsics)]
#![no_std]

extern crate alloc;

pub mod controller;
pub mod drivetrain;
pub mod math;
pub mod devices;
pub mod tracking;
pub mod timer;

pub mod prelude {
    pub use crate::{controller::*, drivetrain::*, math::Vec2, devices::*, tracking::*};
}