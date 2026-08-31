//! Control loops.

mod bang_bang;
mod feedforward;
mod pid;
mod tbh;

use std::time::Duration;

pub use bang_bang::BangBang;
pub use feedforward::{
    ArmFeedforward, ArmFeedforwardSetpoint, ElevatorFeedforward, ElevatorFeedforwardSetpoint,
    MotorFeedforward, MotorFeedforwardSetpoint,
};
pub use pid::{Pid};
pub use tbh::TakeBackHalf;

/// Feedback ("closed-loop") controller.
pub trait Feedback {
    /// Representation of the system's state.
    type State;

    /// Control signal produced by the loop.
    type Signal;

    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(
        &mut self,
        measurement: Self::State,
        setpoint: Self::State,
        dt: Duration,
    ) -> Self::Signal;
}

/// Feedforward ("open-loop") controller.
pub trait Feedforward {
    /// Representation of the system's state.
    type State;

    /// Control signal produced by the loop.
    type Signal;

    /// Updates the feedforward controller's setpoint, producing a new control signal.
    fn update(&mut self, setpoint: Self::State, dt: Duration) -> Self::Signal;
}
