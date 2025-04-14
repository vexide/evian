use core::time::Duration;

use crate::ControlLoop;

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
        Self { kh, tbh: 0.0, integral: 0.0, prev_error: 0.0, }
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
    type Input = f64;
    type Output = f64;

    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        _dt: Duration,
    ) -> Self::Output {
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
