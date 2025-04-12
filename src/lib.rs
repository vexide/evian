#![no_std]

extern crate alloc;

pub mod drivetrain;
pub mod control;
pub mod motion;
pub mod math;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        control::{ControlLoop, Tolerances},
        drivetrain::{
            Drivetrain,
            differential::{Differential, Voltages},
        },
        math::{curve::CubicBezier, Angle, IntoAngle, Vec2},
        tracking::{
            wheeled::{TrackingWheel, WheeledTracking},
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        },
    };
}
