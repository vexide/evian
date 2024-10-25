//! PID Controller
//!
//! The PID controller is one of the most widely used feedback control algorithms in
//! industrial control systems, robotics, and process control. It computes a control signal
//! based on three terms: **proportional**, **integral**, and **derivative**.
//!
//! # Components
//!
//! - The **proportional** term produces an output proportional to the current error,
//!   multiplied by the gain constant `kp`. This provides the main control action:
//!   larger errors result in larger corrective responses and vice versa.
//!
//! - The **integral** term sums the error over time, multiplied by the gain constant `ki`.
//!   This term eliminates *steady-state error* by ensuring that even small errors will
//!   eventually accumulate a large enough response to reach the setpoint. Without integral
//!   action, a system might stabilize with a small, persistent error if the control
//!   signal becomes too small to overcome system friction, gravity, or other external
//!   factors on the system. For example, a motor might need some minimum voltage to start
//!   moving, or a drone might need extra thrust to hover against gravity. The integral term
//!   accumulates over time to provide this additional correction.
//!
//! - The **derivative** term measures the error's change over time, multiplied by the
//!   gain constant `kd`. This provides a damping effect that reduces overshoot and
//!   oscillation by counteracting rapid changes in error. The derivative term helps
//!   anticipate and smooth out the system's response, preventing sudden changes resulting
//!   from large proportional or integral gains.
//!
//! # Tuning
//!
//! Tuning a PID controller requires adjusting `kp`, `ki`, and `kd` to allow the system to reach a setpoint
//! in a reasonable amount of time without oscillations (rapid, unpredictable changes in output).
//!
//! Tuning methods are typically dependent on the application that the PID controller is used
//! in, but a common method is as follows:
//!
//! 1. Start with all gains at zero (`kp = 0.0`, `ki = 0.0`, `kd = 0.0`).
//!
//! 2. Tune proportional gain first:
//!    - Gradually increase `kp` until the system starts to oscillate around the setpoint.
//!    - *Oscillation* occurs when the system reaches and overshoots the setpoint, then repeatedly overadjusts
//!      itself around the setpoint, resulting in a "back-and-fourth" motion around the setpoint.
//!
//! 3. Tune the derivative gain:
//!    - Start with a very small `kd` gain (0.05 × `kp` or less is a safe bet to start with).
//!    - Gradually increase by small increments until oscillations from the proportional term stop occurring.
//!
//! 4. Add integral gain if necessary:
//!    - Integral gain is only necessary if your controller's proportional and derivative terms
//!      become small enough to where they can no longer overcome some external factor (such as friction)
//!      of the system, resulting in what's called *steady-state error*.
//!    - Start with a very small `ki` gain (such as 0.01 × `kp`).
//!    - Increase `ki` slowly until steady-state errors are eliminated within an acceptable time.
//!    - If oscillation occurs, reduce both `ki` and `kp` slightly.
//!
//! Common signs of poor tuning:
//!
//! - Slow response: `kp` is too low.
//! - Excessive overshoot: `kd` is too low or `ki` is too high.
//! - Oscillation: `kp` is too high or `kd` is too low.
//! - Noisy, unpredictable response: `kd` is too high.
//!
//! # Integral Windup (and Mitigations)
//!
//! In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
//! reaches a saturation point preventing the error from decreasing. In this case, integral will rapidly
//! accumulate, causing large and unpredictable control signals. This specific implementation provides
//! two mitigations for integral windup:
//!
//! 1. **Sign-based reset:** When the sign of error changes (in other words, when the controller has
//!    crossed/overshot its target), the integral value is reset to prevent overshoot of the target.
//! 2. **Integration bounds:** An optional `integration_range` value can be passed to the controller,
//!    which defines a range of error where integration will occur. When `|error| > integration_range`,
//!    no integration will occur if used.
use core::time::Duration;

use vexide::prelude::Float;

use super::Feedback;

/// A proportional-integral-derivative (PID) feedback controller with integral windup prevention.
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
    type Error = f64;

    type Output = f64;

    fn update(&mut self, error: Self::Error, dt: Duration) -> Self::Output {
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
