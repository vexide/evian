#![no_std]

extern crate alloc;

pub mod control;
pub mod differential;
pub mod drivetrain;
pub mod math;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        control::{ControlLoop, Tolerances},
        differential::{
            trajectory::{Trajectory, TrajectoryConstraints},
            Differential, Voltages,
        },
        drivetrain::{shared_motors, Drivetrain},
        math::{curve::CubicBezier, Angle, IntoAngle, Vec2},
        tracking::{
            wheeled::{TrackingWheel, WheeledTracking},
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        },
    };
}
