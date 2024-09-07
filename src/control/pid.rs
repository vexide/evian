use vexide::{core::time::Instant, prelude::Float};

use super::MotionController;

/// A proportional-integral-derivative (PID) feedback controller.
///
/// The PID controller is a feedback control algorithm with common applications
/// in industrial control systems, robotics, and analog devices. PID operates on
/// three components - proportional, integral, and derivative.
///
/// # Components
///
/// - The proportional component of a PID controller consists of the error value multiplied
///   by an arbitrary constant `kp`. It is called the proportional component because it increases
///   its output proportional to the error value. As error increases, the proportional component's
///   output increases with it.
///
/// - The integral component of a PID controller describes the accumulation of error over time. It is the
///   sum of all error values multiplied by a constant `ki` fed to this feedback controller when `update`
///   is called. In practice, this component will help to correct for cases where a system slightly undershoots
///   the setpoint ("steady-state error").
///
///   > In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
///   > reaches a saturation point, preventing the error from decreasing. In this case, integral will rapidly
///   > increase, causing an unpredictable (usually much larger than expected) output. This specific implementation
///   > of PID has no guards against integral windup. In cases where this is a problem, a custom implementation
///   > of `MotionController` or simply setting `ki` to `0.0` may be desirable
///
/// - The derivative component represents the change in error over time. The derivative component is the
///   difference between the error given to `update` and the error given to `update` the last time it was
///   called, multiplied by a constant `kd`. In practice, this component will apply a "damping" effect to the
///   controller, preventing sudden jerks or changes to the output.
///
/// # Tuning
///
/// Tuning a PID controller requires adjusting the three constants - `kp`, `ki`, and `kd` to allow the
/// controlled system to reach a setpoint in a reasonable amount of time without oscillations (rapid,
/// unpredictable changes in output). Tuning methods are typically different depending on the application
/// that the PID controller is used in. Typically a tuned `kp` will be much higher than `ki` and `kd`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidController {
    /// The proportional gain constant.
    pub kp: f64,

    /// The integral gain constant.
    pub ki: f64,

    /// The derivative gain constant.
    pub kd: f64,

    pub integral_threshold: f64,

    integral: f64,
    prev_error: f64,
    prev_timestamp: Instant,
}

impl PidController {
    /// Construct a new [`PIDController`] from gain constants.
    pub fn new(gains: (f64, f64, f64), integral_threshold: f64) -> Self {
        Self {
            kp: gains.0,
            ki: gains.1,
            kd: gains.2,
            integral_threshold,
            integral: 0.0,
            prev_error: 0.0,
            prev_timestamp: Instant::now(),
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    pub fn gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    pub fn integral_threshold(&self) -> f64 {
        self.integral_threshold
    }

    /// Sets the PID gains to provided values.
    pub fn set_gains(&mut self, gains: (f64, f64, f64)) {
        self.kp = gains.0;
        self.ki = gains.1;
        self.kd = gains.2;
    }

    pub fn set_integral_threshold(&mut self, threshold: f64) {
        self.integral_threshold = threshold;
    }
}

impl MotionController for PidController {
    type Input = f64;
    type Output = f64;

    fn update(&mut self, error: Self::Input) -> Self::Output {
        let dt = self.prev_timestamp.elapsed();

        if error.abs() < self.integral_threshold {
            self.integral += error;
        }

        if error.signum() != self.prev_error.signum() {
            self.integral = 0.0;
        }

        let derivative = error - self.prev_error;
        self.prev_error = error;

        self.prev_timestamp = Instant::now();

        (error * self.kp)
            + (self.integral * self.ki * dt.as_secs_f64())
            + (derivative * self.kd / dt.as_secs_f64())
    }
}
