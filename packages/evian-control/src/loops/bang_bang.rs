use core::time::Duration;

use crate::loops::ControlLoop;

use super::{Feedback, FeedbackMarker};

/// Bang-bang controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BangBang {
    magnitude: f64,
}

impl BangBang {
    /// Creates a new bang-bang controller with a given output magnitude.
    pub fn new(magnitude: f64) -> Self {
        Self { magnitude }
    }

    /// Returns the controller's output magnitude.
    pub fn magnitude(&self) -> f64 {
        self.magnitude
    }

    /// Sets the controller's output magnitude.
    pub fn set_magnitude(&mut self, magnitude: f64) {
        self.magnitude = magnitude;
    }
}

impl ControlLoop for BangBang {
    type Marker = FeedbackMarker;
    type Input = f64;
    type Output = f64;
}

impl Feedback for BangBang {
    fn update(&mut self, measurement: f64, setpoint: f64, _dt: Duration) -> f64 {
        if measurement < setpoint {
            self.magnitude
        } else {
            0.0
        }
    }
}
