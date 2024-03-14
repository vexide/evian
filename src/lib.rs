#![no_std]
#![allow(internal_features)]
#![feature(core_intrinsics)] // math go zoom

extern crate alloc;

pub mod devices;
pub mod drivetrain;
pub mod math;
pub mod commands;
pub mod tracking;
pub mod controller;

pub mod prelude {
    pub use crate::{
        devices::drive_motors,
        drivetrain::DifferentialDrivetrain,
        math::Vec2,
        tracking::Tracking,
        commands::joystick::{JoystickCommands, JoystickLayout},
    };
}
