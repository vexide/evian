//! Conditional Settling Utility for Commands
//!
//! This module provides utilities for determining when a control system (typically a [`Command`])
//! has "settled" at its target state, signaling that the algorithm has successfully reached its goal.
//! While different control algorithms may define "completeness" in varying ways, this module's
//! [`Tolerances`] struct implements the common approach of settling via *tolerances*.
//!
//! # Tolerances
//!
//! *Tolerances* define an acceptable range of values around a target state. This is useful to define,
//! because physical systems will never achieve their exact target due to real-world factors
//! like friction, sensor noise, and mechanical limitations. Under the [`Tolerances`] struct, a system is considered
//! "settled" when it meets specified error and velocity tolerances for a given duration, after when a timeout is
//! reached.

use core::time::Duration;

use vexide::core::time::Instant;

/// A utility for determining when a control system has stabilized reasonably near its setpoint.
///
/// This struct monitors both position error and velocity to determine if a system has
/// reached and stabilized at its target. It can be configured with tolerances for both
/// error and velocity, a required duration to maintain those tolerances, and an optional
/// timeout for if the target isn't reached in a reasonable amount of time.
///
/// # Settling Logic
///
/// A system is considered settled if either:
/// - The specified timeout has elapsed since the first call to [`Tolerances::is_settled`], OR
/// - Both:
///   1. The error and velocity are within their respective tolerances.
///   2. The system has maintained these tolerances for the specified duration.
///
/// If the system leaves the tolerance window before the duration is met, the tolerance timer resets.
#[derive(Default, Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Tolerances {
    start_timestamp: Option<Instant>,
    tolerance_timestamp: Option<Instant>,
    pub tolerance_duration: Option<Duration>,
    pub error_tolerance: Option<f64>,
    pub velocity_tolerance: Option<f64>,
    pub timeout: Option<Duration>,
}

impl Tolerances {
    /// Creates a new [`Tolerances`] instance with no configured tolerances or timings.
    ///
    /// Until tolerances are configured using the builder methods, all tolerance
    /// checks will pass immediately.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            start_timestamp: None,
            tolerance_timestamp: None,

            tolerance_duration: None,
            error_tolerance: None,
            velocity_tolerance: None,
            timeout: None,
        }
    }

    /// Sets the maximum acceptable error value for settling.
    ///
    /// The error tolerance defines how close to the target position the system
    /// must be to be considered "within tolerance".
    #[must_use]
    pub const fn error_tolerance(&mut self, tolerance: f64) -> Self {
        self.error_tolerance = Some(tolerance);
        *self
    }

    /// Sets the maximum acceptable velocity for settling.
    ///
    /// The velocity tolerance defines how slow the system must be moving to be
    /// considered "stable".
    #[must_use]
    pub const fn velocity_tolerance(&mut self, tolerance: f64) -> Self {
        self.velocity_tolerance = Some(tolerance);
        *self
    }

    /// Sets how long the system must remain within tolerances to be considered settled.
    ///
    /// This duration acts as a "debounce" to ensure the system has truly stabilized
    /// and isn't just passing through the tolerance window momentarily.
    #[must_use]
    pub const fn tolerance_duration(&mut self, duration: Duration) -> Self {
        self.tolerance_duration = Some(duration);
        *self
    }

    /// Sets a maximum duration to wait for settling before forcing completion.
    ///
    /// If this timeout elapses before the system settles normally, this struct
    /// will report as settled regardless of the actual system state. This prevents
    /// commands from hanging indefinitely if settling proves impossible for whatever
    /// reason.
    #[must_use]
    pub const fn timeout(&mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        *self
    }

    /// Checks if the system has settled based on current error and velocity.
    ///
    /// This method should be called periodically (typically in a control loop)
    /// with current system measurements. It will return `true` when either:
    ///
    /// - The specified timeout has elapsed since the first call to this function, OR
    /// - Both:
    ///   1. The error and velocity are within their respective tolerances.
    ///   2. The system has maintained these tolerances for the specified duration.
    /// # Parameters
    ///
    /// * `error` - Difference between the setpoint and measured state of the system.
    /// * `velocity` - Measurement of how fast the system response is changing over time.
    pub fn check(&mut self, error: f64, velocity: f64) -> bool {
        // Initialize timing on first call.
        if self.start_timestamp.is_none() {
            self.start_timestamp = Some(Instant::now());
        }

        // If we have timed out, then we are settled.
        if let Some(timeout) = self.timeout {
            if self.start_timestamp.unwrap().elapsed() > timeout {
                self.tolerance_timestamp = None;
                self.start_timestamp = None;
                return true;
            }
        }

        // Check if we are within the tolerance range for either error and velocity.
        let in_tolerances = self
            .error_tolerance
            .is_none_or(|tolerance| error.abs() < tolerance)
            && self
                .velocity_tolerance
                .is_none_or(|tolerance| velocity.abs() < tolerance);

        if in_tolerances {
            // We are now within tolerance, so we record the timestamp that this occurred if
            // we previously weren't in tolerance.
            if self.tolerance_timestamp.is_none() {
                self.tolerance_timestamp = Some(Instant::now());
            }

            // If we have a tolerance time (required time to be within tolerance to settle), then compare that with
            // the elapsed tolerance timer. If we've been settled for greater than that time, then we are now settled.
            if self
                .tolerance_duration
                .is_none_or(|time| self.tolerance_timestamp.unwrap().elapsed() > time)
            {
                self.tolerance_timestamp = None;
                self.start_timestamp = None;
                return true;
            }
        } else if self.tolerance_timestamp.is_some() {
            self.tolerance_timestamp = None;
        }

        false
    }
}
