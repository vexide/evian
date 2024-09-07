use core::time::Duration;

use vexide::core::time::Instant;


#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Settler {
    start_timestamp: Instant,
    tolerance_timestamp: Option<Instant>,
    tolerance: Option<f64>,
    tolerance_time: Option<Duration>,
    timeout: Option<Duration>,
}

impl Settler {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            start_timestamp: Instant::now(),
            tolerance_timestamp: None,
            tolerance: None,
            tolerance_time: None,
            timeout: None,
        }
    }

    pub fn tolerance(&mut self, tolerance: f64) -> Self {
        self.tolerance = Some(tolerance);
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

    pub fn is_settled(&mut self, error: f64) -> bool {
        if let Some(timeout) = self.timeout {
            if self.start_timestamp.elapsed() < timeout {
                return true;
            }
        }

        if let Some(tolerance) = self.tolerance {
            if error < tolerance {
                if self.tolerance_timestamp.is_none() {
                    self.tolerance_timestamp = Some(Instant::now());
                } else if let Some(tolerance_time) = self.tolerance_time {
                    if self.tolerance_timestamp.unwrap().elapsed() > tolerance_time {
                        return true;
                    }
                }
            } else if self.tolerance_timestamp.is_some() {
                self.tolerance_timestamp = None;
            }
        }

        false
    }
}
