//! # /ˈeɪviɒn/
//!
//! A controls library for vexide.
//!
//! evian is a highly extensible library for controlling mobile robots using the
//! [vexide robotics runtime].
//!
//! [vexide robotics runtime]: https://vexide.dev/
//!
//! # Philosophy
//!
//! Many libraries have been written for competitive robotics in the past. evian isn't
//! the first, and *certainly* isn't the last of its kind. That being said, evian is an
//! attempt to reconcile what we believe to be the best parts of these prior works into
//! something that people can both learn from *and* enjoy using.
//!
//! ## User Friendliness and Extensibility
//!
//! A common issue that many robotics libraries run into is the complexity that comes
//! with making a library *extensible*. As more and more different robot configurations
//! and features are supported, the user friendliness of such libraries is lost, which
//! makes them much harder to approach and easily learn. Furthermore, libraries that
//! aim to solve *literally everyone's problems* often end up becoming monolithic and
//! impossible to maintain as a result.
//!
//! On the opposite end, others have attempted to make libraries as easy to use as
//! possible to the point that they intentionally limit the scope and correctness of
//! their of their features to appeal to new users. While this does succeed in making
//! things more approachable, people actually end up learning *less* in the process,
//! because the highly simplified nature of these types of projects generally discourages
//! any sort of further innovation beyond what "just works already". That's no fun.
//!
//! evian is a *desperate attempt* at a third option—something that people can both
//! extend for their own purposes *and* something that is actually nice to use.
//!
//! ## evian as a Framework
//!
//! evian is first and foremost built to be *extended*. It is designed in a manner
//! that you can provide your own implementations of many "building blocks" while
//! remaining compatible with evian's wider features. You can use your own sensors, your
//! own odometry algorithms and motion control algorithms. You can even write your own
//! drivetrain configuration around evian's [`Drivetrain`] type. The codebase is highly
//! generic with support for this in mind.
//!
//! [`Drivetrain`]: crate::drivetrain::Drivetrain
//!
//! ```
//! pub struct MyLocalization {}
//!
//! impl TracksPosition for MyLocalization {
//!     fn position(&mut self) -> Vec2 {
//!         // Custom position tracking logic!
//!     }
//! }
//!
//! // ...
//!
//! let mut drivetrain = Drivetrain::new(
//!     Differential::new(left_motors, right_motors),
//!     MyLocalization {},
//! );
//! ```
//!
//! In short, you are free to use as much or as little of evian as possible, but in
//! doing so you can make your code compatible with the wider ecosystem.
//!
//! ## evian as a Library
//!
//! evian is of course useable as a general-purpose autonomous library. The [`motion`]
//! module implements many of the commonplace motion control algorithms for autonomous
//! control of your robot. evian makes heavy use of [async rust], as well builder-style
//! modifiers for cleanly composing autonomous routines.
//!
//! ```
//! seeking.move_to_point(dt, (24.0, 24.0)) // Move to point (24, 24) on the field...
//!     .reverse() // ...backwards
//!     .with_angular_kp(0.4) // ...with different PID gains
//!     .without_timeout() // ...with no timeout
//!     .with_linear_output_limit(Motor::V5_MAX_VOLTAGE * 0.75) // ...at 75% of max speed.
//!     .await; // do it!
//! ```
//!
//! [async rust]: https://vexide.dev/docs/async-introduction/
//!
//! In addition, a standard wheeled tracking (odometry) implementation is provided by the
//! [`WheeledTracking`] type in our [`tracking`] module.
//!
//! [`WheeledTracking`]: crate::tracking::wheeled::WheeledTracking
//!
//! Motions in evian are a little "flipped" from what you might be used to in other
//! libraries. Rather than calling motion-related methods on our drivetrain, we instead
//! pass the drivetrain *to the motion*.
//!
//! ```
//! let mut basic = Basic {
//!     linear_controller: LINEAR_PID,
//!     angular_controller: ANGULAR_PID,
//!     linear_tolerances: LINEAR_TOLERANCES,
//!     angular_tolerances: ANGULAR_TOLERANCES,
//!     timeout: Some(Duration::from_secs(10)),
//! };
//!
//! // Drive forwards.
//! basic.drive_distance(&mut drivetrain, 24.0).await;
//! ```
//!
//! This is done to allow for an extremely easy way to create custom motions. In fact, "motions"
//! in evian are nothing more than simple `async` functions that mutably borrow your drivetrain
//! for a period of time.
//!
//! ```
//! /// A motion algorithm for differential drivetrains.
//! ///
//! /// Requires a tracking system that records robot position and robot heading (orientation).
//! pub async fn fly(
//!     drivetrain: &mut Drivetrain<Differential, impl TracksPosition + TracksHeading>,
//! ) {
//!     loop {
//!         // ...
//!
//!         sleep(Duration::from_millis(5)).await;
//!     }
//! }
//! ```

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
