use core::time::Duration;

use crate::loops::{ControlLoop, Feedback};

/// Take-back-half flywheel velocity controller.
pub struct TakeBackHalf {
    kh: f64,
    tbh: f64,
    integral: f64,
    prev_error: f64,
}

impl TakeBackHalf {
    /// Creates a new TBH controller.
    pub fn new(kh: f64) -> Self {
        Self {
            kh,
            tbh: 0.0,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    /// Returns the controller's integral gain (`kh`).
    pub fn kh(&self) -> f64 {
        self.kh
    }

    /// Sets the controller's integral gain (`kh`).
    pub fn set_kh(&mut self, kh: f64) {
        self.kh = kh;
    }
}

impl ControlLoop for TakeBackHalf {
    type State = f64;
    type Signal = f64;
}

impl Feedback for TakeBackHalf {
    fn update(&mut self, measurement: f64, setpoint: f64, _dt: Duration) -> f64 {
        let error = setpoint - measurement;

        self.integral += error * self.kh;

        if error * self.prev_error <= 0.0 {
            self.integral = 0.5 * (self.integral + self.tbh);
            self.tbh = self.integral;
        }

        self.prev_error = error;

        self.tbh
    }
}
