use core::time::Duration;

use vexide::core::time::Instant;

#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Settler {
    start_timestamp: Instant,
    tolerance_timestamp: Option<Instant>,
    tolerance_time: Option<Duration>,
    error_tolerance: Option<f64>,
    velocity_tolerance: Option<f64>,
    timeout: Option<Duration>,
}

impl Settler {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            start_timestamp: Instant::now(),
            tolerance_timestamp: None,
            error_tolerance: None,
            velocity_tolerance: None,
            tolerance_time: None,
            timeout: None,
        }
    }

    pub fn error_tolerance(&mut self, tolerance: f64) -> Self {
        self.error_tolerance = Some(tolerance);
        *self
    }

    pub fn velocity_tolerance(&mut self, tolerance: f64) -> Self {
        self.velocity_tolerance = Some(tolerance);
        *self
    }

    pub fn tolerance_time(&mut self, tolerance_time: Duration) -> Self {
        self.tolerance_time = Some(tolerance_time);
        *self
    }

    pub fn timeout(&mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        *self
    }

    pub fn is_settled(&mut self, error: f64, velocity: f64) -> bool {
        // If we have timed out, then we are settled.
        if let Some(timeout) = self.timeout {
            if self.start_timestamp.elapsed() > timeout {
                return true;
            }
        }

        // Check if we are within the tolerance range for either error and velocity.
        let in_tolerances = self
            .error_tolerance
            .is_none_or(|tolerance| error < tolerance)
            && self
                .velocity_tolerance
                .is_none_or(|tolerance| velocity < tolerance);

        if in_tolerances {
            // We are now within tolerance, so we record the timestamp that this occurred if
            // we previously weren't in tolerance.
            if self.tolerance_timestamp.is_none() {
                self.tolerance_timestamp = Some(Instant::now());
            }

            // If we have a tolerance time (required time to be within tolerance to settle), then compare that with
            // the elapsed tolerance timer. If we've been settled for greater than that time, then we are now settled.
            if self.tolerance_time.is_none_or(|time| self.tolerance_timestamp.unwrap().elapsed() > time) {
                return true;
            }
        } else if self.tolerance_timestamp.is_some() {
            self.tolerance_timestamp = None;
        }

        false
    }
}
