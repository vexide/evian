#![no_std]

extern crate alloc;

pub mod control;
pub mod drivetrain;
pub mod math;
pub mod motion;
pub mod tracking;

pub mod prelude {
    pub use crate::{
        control::{ControlLoop, Tolerances},
        drivetrain::{
            differential::{Differential, Voltages},
            Drivetrain,
        },
        math::{curve::CubicBezier, Angle, IntoAngle, Vec2},
        tracking::{
            wheeled::{TrackingWheel, WheeledTracking},
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        },
    };
}
