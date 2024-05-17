#![no_std]
#![allow(internal_features)]
#![feature(core_intrinsics)] // math go zoom

extern crate alloc;

pub mod commands;
pub mod controller;
pub mod devices;
pub mod drivetrain;
pub mod math;
pub mod tracking;
pub mod settle;

pub mod prelude {
    pub use crate::{
        commands::joystick::{JoystickCommands, JoystickLayout},
        devices::drive_motors,
        drivetrain::DifferentialDrivetrain,
        math::Vec2,
        tracking::Tracking,
    };
}
