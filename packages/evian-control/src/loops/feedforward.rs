use evian_math::Angle;

use crate::loops::ControlLoop;

use super::{Feedforward, FeedforwardMarker};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedforwardSetpoint {
    pub velocity: f64,
    pub acceleration: f64,
}

/// Ideal DC motor feedforward controller.
///
/// This is a open-loop velocity controller that computes the voltage to
/// maintain an idealized DC motor at a certain velocity and acceleration.
///
/// The controller is implemented according to the following model:
///
/// `V = Kₛ sign(ω) + Kᵥ ω + Kₐ α`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedforward {
    ks: f64,
    kv: f64,
    ka: f64,
}

impl MotorFeedforward {
    /// Creates a new [`MotorFeedforward`] with the given constants and target.
    ///
    /// # Parameters
    ///
    /// - `ks` - Feedforward constant for static friction compensation.
    /// - `kv` - Feedforward constant for velocity compensation.
    /// - `ka` - Feedforward constant for acceleration compensation.
    /// - `target_acceleration` - Feedforward constant for the target acceleration.
    pub const fn new(ks: f64, kv: f64, ka: f64) -> Self {
        Self { ks, kv, ka }
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

impl ControlLoop for MotorFeedforward {
    type Marker = FeedforwardMarker;
    type Input = MotorFeedforwardSetpoint;
    type Output = f64;
}

impl Feedforward for MotorFeedforward {
    fn update(&mut self, setpoint: MotorFeedforwardSetpoint, _dt: core::time::Duration) -> f64 {
        self.ks * setpoint.velocity.signum()
            + self.kv * setpoint.velocity
            + self.ka * setpoint.acceleration
    }
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmFeedforwardSetpoint {
    pub position: Angle,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmFeedforward {
    ks: f64,
    kg: f64,
    kv: f64,
    ka: f64,
}

impl ArmFeedforward {
    pub const fn new(ks: f64, kg: f64, kv: f64, ka: f64) -> Self {
        Self { ks, kg, kv, ka }
    }

    /// Get the current PID gains as a tuple (`ks`, `kg`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (f64, f64, f64, f64) {
        (self.ks, self.kg, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> f64 {
        self.ks
    }

    #[must_use]
    pub const fn kg(&self) -> f64 {
        self.kg
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
    pub const fn set_constants(&mut self, ks: f64, kg: f64, kv: f64, ka: f64) {
        self.ks = ks;
        self.kg = kg;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: f64) {
        self.ks = ks;
    }

    pub const fn set_kg(&mut self, ks: f64) {
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

impl ControlLoop for ArmFeedforward {
    type Marker = FeedforwardMarker;
    type Input = ArmFeedforwardSetpoint;
    type Output = f64;
}

impl Feedforward for ArmFeedforward {
    fn update(&mut self, setpoint: ArmFeedforwardSetpoint, _dt: core::time::Duration) -> f64 {
        self.kg * setpoint.position.cos()
            + self.ks * setpoint.velocity.signum()
            + self.kv * setpoint.velocity
            + self.ka * setpoint.acceleration
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevatorFeedforwardSetpoint {
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevatorFeedforward {
    ks: f64,
    kg: f64,
    kv: f64,
    ka: f64,
}

impl ElevatorFeedforward {
    pub const fn new(ks: f64, kg: f64, kv: f64, ka: f64) -> Self {
        Self { ks, kg, kv, ka }
    }

    /// Get the current PID gains as a tuple (`ks`, `kg`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (f64, f64, f64, f64) {
        (self.ks, self.kg, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> f64 {
        self.ks
    }

    #[must_use]
    pub const fn kg(&self) -> f64 {
        self.kg
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
    pub const fn set_constants(&mut self, ks: f64, kg: f64, kv: f64, ka: f64) {
        self.ks = ks;
        self.kg = kg;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: f64) {
        self.ks = ks;
    }

    pub const fn set_kg(&mut self, ks: f64) {
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

impl ControlLoop for ElevatorFeedforward {
    type Marker = FeedforwardMarker;
    type Input = ElevatorFeedforwardSetpoint;
    type Output = f64;
}

impl Feedforward for ElevatorFeedforward {
    fn update(&mut self, setpoint: ElevatorFeedforwardSetpoint, _dt: core::time::Duration) -> f64 {
        self.kg
            + self.ks * setpoint.velocity.signum()
            + self.kv * setpoint.velocity
            + self.ka * setpoint.acceleration
    }
}
