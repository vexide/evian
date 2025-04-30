//! Control loops.

mod bang_bang;
mod cascade;
mod pid;
mod tbh;
pub mod feedforward;

use core::time::Duration;

pub use bang_bang::BangBang;
pub use cascade::Cascade;
pub use pid::{AngularPid, Pid};
pub use tbh::TakeBackHalf;
pub use feedforward::MotorFeedforward;

mod sealed {
    pub trait ControlLoopMarker {}
}

/// Marker type for the [`ControlLoop`] trait indicating that the type will implement
/// the [`Feedback`] trait.
/// 
/// This type exists to enforce that the [`Feedback`] and [`Feedforward`] traits are
/// mutually exclusive. Implementors of the [`Feedback`] trait are required to set
/// `type Marker = `FeedbackMarker;` in their [`ControlLoop`] implementation.
pub struct FeedbackMarker(());

/// Marker type for the [`ControlLoop`] trait indicating that the type will implement
/// the [`Feedforward`] trait.
/// 
/// This type exists to enforce that the [`Feedback`] and [`Feedforward`] traits are
/// mutually exclusive. Implementors of the [`Feedforward`] trait are required to set
/// `type Marker = `FeedforwardMarker;` in their [`ControlLoop`] implementation.
pub struct FeedforwardMarker(());

impl sealed::ControlLoopMarker for FeedbackMarker {}
impl sealed::ControlLoopMarker for FeedforwardMarker {}

/// Functionality for a generic control loop.
///
/// Control loops are fundamental to control systems, where a controller
/// continually adjusts its output (the "control signal") to drive a control
/// system to reach its desired value (the "setpoint").
pub trait ControlLoop {
    /// A marker type (must be either [`FeedbackMarker`] or [`FeedforwardMarker`])
    /// that indicates whether this type will implement [`Feedback`] or [`Feedforward`].
    /// 
    /// This exists to enforce that the [`Feedback`] and [`Feedforward`] traits are mutually
    /// exclusive implementations.
    type Marker: sealed::ControlLoopMarker;

    /// Representation of the system's state.
    type Input;

    /// Control signal produced by the loop.
    type Output;
}

/// Feedback ("closed-loop") controller.
pub trait Feedback: ControlLoop<Marker = FeedbackMarker> {
    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}

/// Feedforward ("open-loop") controller.
pub trait Feedforward: ControlLoop<Marker = FeedforwardMarker> {
    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(&mut self, setpoint: Self::Input, dt: Duration) -> Self::Output;
}
