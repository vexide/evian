//! Control Theory Primitives
//!
//! This module provides basic building-blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

use core::time::Duration;

pub mod pid;

/// A trait for implementing feedback control of a system.
///
/// Feedback controllers are algorithms that attempt to reach a desired state (setpoint) by:
///
/// 1. Measuring the difference (error) between the current and the desired state.
/// 2. Computing corrective control signals that drive the system toward the desired setpoint
///    to minimize error in the system.
/// 3. Continuously adjusting based on new measurements of the system.
///
/// This is a form of "closed-loop" control, which refers to how the controller forms a loop
/// by continuously measuring the system's state and "feeding" the measurement back into the
/// controller to adjust for a new, more desirable measurement that minimizes on error.
///
/// # Example
///
/// Simple P (proportional only) controller:
///
/// ```
/// use core::time::Duration;
///
/// struct ProportionalController {
///     gain: f64,
/// }
///
/// impl Feedback for ProportionalController {
///     type Error = f64;
///     type Output = f64;
///
///     fn update(&mut self, error: Self::Error, _dt: Duration) -> Self::Output {
///         self.gain * error
///     }
/// }
/// ```
pub trait Feedback {
    /// The controller's input (error) type.
    ///
    /// Error describes the difference between the setpoint and a measurement ("process value")
    /// of the system.
    type Error;

    /// The controller's output type.
    ///
    /// This is commonly known as a "control signal", and is a value that will ideally decrease
    /// the error of the system after being applied.
    type Output;

    /// Computes a new control signal given a new error measurement.
    ///
    /// This function also requires a `dt` ("delta time") value to be passed, which informs the
    /// controller of the amount of time that has elapsed since the last control signal was computed.
    fn update(&mut self, error: Self::Error, dt: Duration) -> Self::Output;
}
