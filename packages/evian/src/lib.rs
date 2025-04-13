//! # evian

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

#[doc(inline)]
#[cfg(feature = "control")]
pub use evian_control as control;

#[doc(inline)]
#[cfg(feature = "drivetrain")]
pub use evian_drivetrain as drivetrain;

#[doc(inline)]
#[cfg(feature = "math")]
pub use evian_math as math;

#[doc(inline)]
#[cfg(feature = "motion")]
pub use evian_motion as motion;

#[doc(inline)]
#[cfg(feature = "tracking")]
pub use evian_tracking as tracking;

/// Commonly used features of evian.
///
/// This module is meant to be glob imported.
pub mod prelude {
    #[cfg(feature = "drivetrain")]
    pub use crate::drivetrain::{
        differential::{Differential, Voltages},
        shared_motors, Drivetrain,
    };
    #[cfg(feature = "math")]
    pub use crate::math::{curve::CubicBezier, Angle, IntoAngle, Vec2};
    #[cfg(feature = "control")]
    pub use crate::control::Tolerances;
    #[cfg(feature = "tracking")]
    pub use crate::tracking::{
        wheeled::{TrackingWheel, WheeledTracking},
        TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    };
}
