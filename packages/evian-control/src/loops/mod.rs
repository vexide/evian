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
/// continually adjusts its output to drive a control system to its desired value.
///
/// This trait defines the core behavior of such controllers, which take a `setpoint`
/// (the desired target value of the system) and optionally a `measurement` of the
/// system (recorded through a sensor or similar). The controller then computes an
/// output intended to minimize the system's *error*, which is the difference between
/// the `measurement` and the `setpoint`.
///
/// # Examples
///
/// This example shows an implementation of a basic [proportional feedback controller],
/// which scales its output based on the system's error. Measurements further from the
/// setpoint will yield larger control signals.
///
/// ```
/// pub struct PController {
///     pub kp: f64,
/// }
///
/// impl ControlLoop for PController {
///     type Input = f64;
///     type Output = f64;
///
///     fn update(
///         &mut self,
///         measurement: f64,
///         setpoint: f64,
///         _dt: Duration,
///     ) -> f64 {
///         (setpoint - measurement) * self.kp
///     }
/// }
/// ```
///
/// [proportional feedback controller]: https://en.wikipedia.org/wiki/Proportional_control
pub trait ControlLoop {
    /// The type of input measurements and setpoints that this controller takes.
    type Input;

    /// The type of output (control signal) this controller produces.
    type Output;

    /// Updates the control loop with the latest `measurement` and `setpoint` for the
    /// system, producing a corresponding control signal.
    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}
