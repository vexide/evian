use core::time::Duration;

use vexide::prelude::Float;

use super::Feedback;

/// A proportional-integral-derivative (PID) feedback controller with windup prevention.
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
///
/// # Integral Windup Mitigations
///
/// In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
/// reaches a saturation point preventing the error from decreasing. In this case, integral will rapidly
/// increase, causing an unpredictable (usually much larger than expected) output. This specific
/// implementation provides two mitigations for integral windup:
///
/// 1. When the sign of error changes (in other words, when the controller has crossed/overshot its target), the
///    integral value is reset to prevent overshoot of the target.
/// 2. An optional `integration_range` value can be passed to the controller, which defines a range of error where
///    integration will occur. When `|error| > integration_range`, no integration will occur if used.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integration_range: Option<f64>,

    integral: f64,
    prev_error: f64,
}

impl Pid {
    /// Construct a new PID controller from gain constants and an optional integration range.
    pub fn new(kp: f64, ki: f64, kd: f64, integration_range: Option<f64>) -> Self {
        Self {
            kp,
            ki,
            kd,
            integration_range,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    pub fn gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    pub fn kp(&self) -> f64 {
        self.kp
    }

    pub fn ki(&self) -> f64 {
        self.ki
    }

    pub fn kd(&self) -> f64 {
        self.kd
    }

    pub fn integration_range(&self) -> Option<f64> {
        self.integration_range
    }

    /// Sets the PID gains to provided values.
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_kp(&mut self, kp: f64) {
        self.kp = kp;
    }

    pub fn set_ki(&mut self, ki: f64) {
        self.ki = ki;
    }

    pub fn set_kd(&mut self, kd: f64) {
        self.kd = kd;
    }

    pub fn set_integration_range(&mut self, range: Option<f64>) {
        self.integration_range = range;
    }
}

impl Feedback for Pid {
    type Input = f64;
    type Output = f64;

    fn update(
        &mut self,
        measurement: Self::Output,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output {
        // Calculate error (difference between our setpoint and measured process value).
        let error = setpoint - measurement;

        // If an integration range is used and we are within it, add to the integral.
        // If we are outside of the range, or if we have crossed the setpoint, reset integration.
        if self
            .integration_range
            .is_none_or(|range| error.abs() < range)
            && error.signum() == self.prev_error.signum()
        {
            self.integral += error * dt.as_secs_f64();
        } else {
            self.integral = 0.0;
        }

        // Calculate derivative (change in error / change in time)
        let derivative = (error - self.prev_error) / dt.as_secs_f64();
        self.prev_error = error;

        // Control signal = error * kp + integral + ki + derivative * kd.
        (error * self.kp) + (self.integral * self.ki) + (derivative * self.kd)
    }
}
