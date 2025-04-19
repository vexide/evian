//! # evian
//!
//! Controls library for vexide.

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
    #[cfg(feature = "control")]
    pub use crate::control::Tolerances;
    #[cfg(feature = "drivetrain")]
    pub use crate::drivetrain::{
        Drivetrain,
        differential::{Differential, Voltages},
        shared_motors,
    };
    #[cfg(feature = "math")]
    pub use crate::math::{Angle, IntoAngle, Vec2, curve::CubicBezier};
    #[cfg(feature = "tracking")]
    pub use crate::tracking::{
        TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
        wheeled::{TrackingWheel, WheeledTracking},
    };
}
