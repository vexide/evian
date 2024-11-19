#![no_std]

extern crate alloc;

pub mod control;
pub mod differential;
pub mod math;
pub mod settler;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        control::{
            pid::{AngularPid, Pid},
            ControlLoop,
        },
        differential::{shared_motors, DifferentialDrivetrain, SharedMotors, Voltages},
        math::{Angle, IntoAngle, Vec2},
        settler::Settler,
        tracking::{parallel_wheel::ParallelWheelTracking, wheel::TrackingWheel, Tracking},
    };
}
