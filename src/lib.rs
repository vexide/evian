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
        control::{pid::Pid, Feedback, Feedforward},
        differential::{commands::basic::BasicCommands, drivetrain::DifferentialDrivetrain},
        math::vec2::Vec2,
        settler::Settler,
        tracking::{parallel_wheel::ParallelWheelTracking, wheel::TrackingWheel, Tracking},
    };
}
