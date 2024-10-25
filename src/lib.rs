#![no_std]

extern crate alloc;

pub mod command;
pub mod control;
pub mod differential;
pub mod math;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        command::settler::Settler,
        control::{pid::Pid, Feedback},
        differential::{
            commands::basic::BasicCommands, drive_motors, DifferentialDrivetrain, DriveMotors,
        },
        math::vec2::Vec2,
        tracking::{parallel_wheel::ParallelWheelTracking, wheel::TrackingWheel, Tracking},
    };
}
