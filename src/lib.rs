#![no_std]

extern crate alloc;

pub mod command;
pub mod control;
pub mod differential;
pub mod math;
pub mod settler;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        control::{
            Feedback,
            Feedforward,
            pid::Pid,
        },
        differential::{
            drivetrain::DifferentialDrivetrain,
            commands::basic::BasicMotions,
        },
        tracking::{
            Tracking,
            parallel_wheel::ParallelWheelTracking,
            wheel::TrackingWheel,
        },
        settler::Settler,
        math::vec2::Vec2,
    };
}