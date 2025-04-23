//! Control loops.

mod bang_bang;
mod feedforward;
mod pid;
mod tbh;

use core::time::Duration;

pub use bang_bang::BangBang;
pub use feedforward::DcMotorFeedforward;
pub use pid::{AngularPid, Pid};
pub use tbh::TakeBackHalf;

/// Functionality for a generic control loop.
///
/// Control loops are fundamental to control systems, where a controller
/// continually adjusts its output (the "control signal") to drive a control system to
/// reach its desired value (the "setpoint").
pub trait ControlLoop {
    /// Representation of the system's state.
    type State;

    /// Control signal produced by the loop.
    type Signal;
}

/// Feedback ("closed-loop") controller.
pub trait Feedback: ControlLoop {
    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(
        &mut self,
        measurement: Self::State,
        setpoint: Self::State,
        dt: Duration,
    ) -> Self::Signal;
}

/// Feedforward ("open-loop") controller.
pub trait Feedforward: ControlLoop {
    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(&mut self, setpoint: Self::State, dt: Duration) -> Self::Signal;
}
