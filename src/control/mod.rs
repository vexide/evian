pub mod pid;
pub mod settler;

/// A closed-loop feedback controller.
///
/// At its core, a feedback controller is a simple function that produces an output value
/// given a desired value (a "setpoint") and a measurement of a system's current state. The goal
/// of a feedback controller is to stabilize the system's measured state to match the setpoint as
/// close as possible.
pub trait MotionController: Send + Sync + 'static {
    type Input;
    type Output;

    /// Produce an output value given an `error` value, which is the difference between the measured state
    /// and the desired state (setpoint).
    ///
    /// # Example
    ///
    /// ```
    /// /// A basic proportional controller that multiplies the error value by a constant (2.0).
    /// /// This effectively means that the correction will increase proportional to the growth
    /// /// of the error. A high error value will produce a higher output than a lower error.
    /// fn update(&mut self, error: f64) -> f64 {
    ///     error * 2.0
    /// }
    /// ```
    fn update(&mut self, error: Self::Input) -> Self::Output;
}
