use core::time::Duration;

/// A closed-loop feedback controller.
///
/// At its core, a feedback controller is a simple function that produces an output value
/// given a desired value (a "setpoint") and a measurement of a system's current state. The goal
/// of a feedback controller is to stabilize the system's measured state to match the setpoint as
/// close as possible.
pub trait FeedbackController: Send + Sync + 'static {
    
    /// Produce an output value given an `error` value, which is the difference between the measured state
    /// and the desired state (setpoint).
    ///
    /// # Example
    ///
    /// ```
    /// /// A basic proportional controller that multiplies the error value by a constant.
    /// /// This effectively means that the correction will increase proportional to the growth
    /// /// of the error. A high error value will produce a higher output than a lower error.
    /// fn update(&mut self, error: f64) -> f64 {
    /// 	error * 2.0
    /// }
    /// ```
    fn update(&mut self, error: f64, dt: Duration) -> f64;
}

/// A proportional-integral-derivative (PID) feedback controller.
///
/// The PID controller is a feedback control algorithm with common applications
/// in industrial control systems, robotics, and analog devices. PID operates on
/// three components - proportional, integral, and derivative.
///
/// # Components
///
/// - The proportional component of a PID controller consists of the error value multiplied
/// by an arbitrary constant `kp`. It is called the proportional component because it increases
/// its output proportional to the error value. As error increases, the proportional component's
/// output increases with it.
///
/// - The integral component of a PID controller describes the accumulation of error over time. It is the
/// sum of all error values multiplied by a constant `ki` fed to this feedback controller when `update`
/// is called. In practice, this component will help to correct for cases where a system slightly undershoots
/// the setpoint ("steady-state error").
///
/// 	> In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
/// 	> reaches a saturation point, preventing the error from decreasing. In this case, integral will rapidly
/// 	> increase, causing an unpredictable (usually much larger than expected) output. This specific implementation
/// 	> of PID has no guards against integral windup. In cases where this is a problem, a custom implementation
/// 	> of `FeedbackController` or simply setting `ki` to `0.0` may be desirable
///
/// - The derivative component represents the change in error over time. The derivative component is the
/// difference between the error given to `update` and the error given to `update` the last time it was
/// called, multiplied by a constant `kd`. In practice, this component will apply a "damping" effect to the
/// controller, preventing sudden jerks or changes to the output.
///
/// # Tuning
///
/// Tuning a PID controller requires adjusting the three constants - `kp`, `ki`, and `kd` to allow the
/// controlled system to reach a setpoint in a reasonable amount of time without oscillations (rapid,
/// unpredictable changes in output). Tuning methods are typically different depending on the application
/// that the PID controller is used in. Typically a tuned `kp` will be much higher than `ki` and `kd`.
#[derive(Clone, PartialEq, Debug, Copy, Default)]
pub struct PIDController {
    /// The proportional gain constant.
    pub kp: f64,

    /// The integral gain constant.
    pub ki: f64,

    /// The derivative gain constant.
    pub kd: f64,

    integral: f64,
    previous_error: f64,
}

impl PIDController {
    /// Construct a new [`PIDController`] from gain constants.
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    pub fn gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Sets the PID gains to provided values.
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}

impl FeedbackController for PIDController {
    fn update(&mut self, error: f64, dt: Duration) -> f64 {
        self.integral += error;
        
        let derivative = error - self.previous_error;
        self.previous_error = error;

        (error * self.kp) + (self.integral * self.ki * dt.as_secs_f64()) + (derivative * self.kd / dt.as_secs_f64())
    }
}