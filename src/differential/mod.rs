use vexide::core::float::Float;

pub mod commands;
pub mod drivetrain;

/// Left/Right Motor Voltages
///
/// Used as the standard output of a [`Command`] when working with the [`DifferentialDrivetrain`]
/// struct.
///
/// This struct is additionally a [`Command`] in itself, and can be used to run a drivetrain at a
/// fixed voltage.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Voltages(pub f64, pub f64);

impl Voltages {
    /// Normalizes the ratio of voltages between two motors.
    ///
    /// If either motor is over a `max_voltage`, limit both voltages to preserve
    /// the ratio between left and right power.
    pub fn normalized(&self, max: f64) -> Self {
        let larger_voltage = self.0.abs().max(self.1.abs()) / max;

        let mut voltages = *self;

        if larger_voltage > 1.0 {
            voltages.0 /= larger_voltage;
            voltages.1 /= larger_voltage;
        }

        voltages
    }

    pub fn left(&self) -> f64 {
        self.0
    }

    pub fn right(&self) -> f64 {
        self.1
    }
}

impl From<(f64, f64)> for Voltages {
    fn from(tuple: (f64, f64)) -> Self {
        Self(tuple.0, tuple.1)
    }
}
