#![no_std]

extern crate alloc;

pub mod command;
pub mod control;
pub mod drivetrain;
pub mod math;
pub mod tracking;
pub mod trajectory;

pub mod prelude {
    pub use crate::{
        command::Settler,
        control::{
            pid::{AngularPid, Pid},
            ControlLoop,
        },
        drivetrain::{
            differential::{shared_motors, Differential, Voltages},
            Drivetrain,
        },
        math::{Angle, IntoAngle, Vec2},
        tracking::{
            wheeled::{ParallelWheelTracking, PerpendicularWheelTracking, TrackingWheel},
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        },
    };
}
