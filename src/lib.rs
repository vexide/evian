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
            Differential, DifferentialVoltages,
        },
        drivetrain::{shared_motors, Drivetrain},
        math::{curve::CubicBezier, Angle, IntoAngle, Vec2},
        tracking::{
            wheeled::{ParallelWheelTracking, PerpendicularWheelTracking, TrackingWheel},
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        },
    };
}
