use core::time::Duration;

use crate::ControlLoop;

/// Bang-bang controller.
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
    type Input = f64;
    type Output = f64;

    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        _dt: Duration,
    ) -> Self::Output {
        if measurement < setpoint {
            self.magnitude
        } else {
            0.0
        }
    }
}
