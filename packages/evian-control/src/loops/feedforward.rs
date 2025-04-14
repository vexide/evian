use crate::loops::ControlLoop;

/// Simple DC motor feedforward controller.
///
/// This is a open-loop velocity controller that computes the voltage to
/// maintain an idealized DC motor in a certain state.
///
/// The controller is implemented according to the following model:
///
/// `V = Kₛ sign(ω) + Kᵥ ω + Kₐ α``
pub struct DcMotorFeedforward {
    ks: f64,
    kv: f64,
    ka: f64,

    target_acceleration: f64,
}

impl DcMotorFeedforward {
    /// Creates a new [`FeedforwardMotorController`] with the given constants and target.
    ///
    /// # Parameters
    ///
    /// - `ks` - Feedforward constant for static friction compensation.
    /// - `kv` - Feedforward constant for velocity compensation.
    /// - `ka` - Feedforward constant for acceleration compensation.
    /// - `target_acceleration` - Feedforward constant for the target acceleration.
    pub fn new(ks: f64, kv: f64, ka: f64, target_acceleration: f64) -> Self {
        Self {
            ks,
            kv,
            ka,
            target_acceleration,
        }
    }

    /// Get the current PID gains as a tuple (`ks`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (f64, f64, f64) {
        (self.ks, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> f64 {
        self.ks
    }

    /// Returns the controller's velocity constant (`kv`).
    #[must_use]
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the controller's acceleration constant (`kd`).
    #[must_use]
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Sets the PID gains to provided values.
    pub const fn set_constants(&mut self, ks: f64, kv: f64, ka: f64) {
        self.ks = ks;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: f64) {
        self.ks = ks;
    }

    /// Sets the controller's velocity constant (`kv`).
    pub const fn set_kv(&mut self, kv: f64) {
        self.kv = kv;
    }

    /// Sets the controller's acceleration constant (`ka`).
    pub const fn set_ka(&mut self, ka: f64) {
        self.ka = ka;
    }
}

impl ControlLoop for DcMotorFeedforward {
    type Input = f64;
    type Output = f64;

    fn update(
        &mut self,
        _measurement: Self::Input,
        setpoint: Self::Input,
        _dt: core::time::Duration,
    ) -> Self::Output {
        self.ks * setpoint.signum() + self.kv * setpoint + self.ka * self.target_acceleration
    }
}
