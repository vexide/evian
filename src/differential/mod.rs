//! Differential Drivetrains
//!
//! This module provides support for drivetrains configured in a differential (left/right) wheel
//! configuration.
//!
//! # Overview
//!
//! A differential drivetrain (also called a **tank drive** or **skid-steer**) is a robot whose
//! movement is controlled by two independently driven sets of wheels on the left and right sides
//! of its chassis. The system operates by adjusting the speed and direction of the left and right
//! motors, enabling the robot to drive straight or execute turns.
//!
//! This module provides motor control through the [`Differential`] and [`DifferentialVoltages`],
//! motion control and algorithms through the [`motion`] module, and 2D trajectory generation and
//! motion profiling through the [`trajectory`] module.

pub mod motion;
pub mod trajectory;

use vexide::devices::smart::motor::MotorError;

use crate::drivetrain::SharedMotors;

/// A collection of motors mounted in a differential (left/right) configuration.
///
/// A differential drivetrain (also called a **tank drive** or **skid-steer**) is a robot whose movement is
/// controlled by two independently driven sets of wheels on the left and right sides of its
/// chassis. The system operates by adjusting the speed and direction of the left and right
/// motors, enabling the robot to drive straight or execute turns.
///
/// - If both sets of motors move at the same speed, the robot moves straight.
/// - If one set of motors moves faster than the other, the robot will turn.
/// - If the motors on one side move forward while the other side moves backward, the robot will rotate in place.
///
/// Differential drivetrains are *nonholonomic*, meaning they cannot strafe laterally.
pub struct Differential {
    left: SharedMotors,
    right: SharedMotors,
}

impl Differential {
    /// Creates a new drivetrain with the provided left/right motors.
    ///
    /// Motors created with the [`shared_motors`] macro may be safely cloned, as they are wrapped
    /// in an [`Arc`] to allow sharing across tasks and between the drivetrain and its tracking
    /// instance if needed.
    ///
    /// [`shared_motors`]: crate::drivetrain::SharedMotors
    /// [`Arc`]: alloc::arc::Arc
    ///
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     shared_motors![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ],
    ///     shared_motors![
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    /// ```
    pub const fn new(left: SharedMotors, right: SharedMotors) -> Self {
        Self { left, right }
    }

    /// Sets the voltage of the left and right motors.
    ///
    /// # Errors
    ///
    /// See [`Motor::set_voltage`].
    ///
    /// [`Motor::set_voltage`]: vexide::devices::smart::motor::Motor::set_voltage
    ///
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     shared_motors![
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///     ],
    ///     shared_motors![
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    ///
    /// motors.set_voltages(DifferentialVoltages(12.0, 12.0))?;
    /// ```
    pub fn set_voltages(
        &mut self,
        voltages: impl Into<DifferentialVoltages>,
    ) -> Result<(), MotorError> {
        let voltages = voltages.into();
        let mut rtn = Ok(());

        for motor in self.left.borrow_mut().iter_mut() {
            let result = motor.set_voltage(voltages.left());

            if result.is_err() {
                rtn = result;
            }
        }

        for motor in self.right.borrow_mut().iter_mut() {
            let result = motor.set_voltage(voltages.right());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}

/// Left/Right Motor Voltages
///
/// These voltages are used to control a [`Differential`] motor configuration. They describe
/// the voltages of the respective left and right motors.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct DifferentialVoltages(pub f64, pub f64);

impl DifferentialVoltages {
    /// Creates a [`DifferentialVoltages`] instance from a provided linear and angular voltage.
    ///
    /// # Examples
    ///
    /// ```
    /// let voltages = DifferentialVoltages::from_arcade(5.0, 2.0);
    /// assert_eq!(voltages, DifferentialVoltages(7.0, 3.0));
    /// ```
    #[must_use]
    pub const fn from_arcade(linear: f64, angular: f64) -> Self {
        Self(linear + angular, linear - angular)
    }

    /// Returns [`DifferentialVoltages`] that are less than a provided `max` value while
    /// preserving the ratio between the original left and right values.
    ///
    /// If either motor is over a `max_voltage`, both values will be decresed by the amount
    /// that is "oversaturated" to preserve the ratio between left and right power.
    ///
    /// # Examples
    ///
    /// ```
    /// let voltages = DifferentialVoltages::from_arcade(12.0, 13.0).normalized(12.0);
    ///
    /// assert_eq!(voltages, voltages.left() <= 12.0);
    /// assert_eq!(voltages, voltages.right() <= 12.0);
    /// ```
    #[must_use]
    pub fn normalized(&self, max: f64) -> Self {
        let larger_magnitude = self.0.abs().max(self.1.abs()) / max;

        let mut voltages = *self;

        if larger_magnitude > 1.0 {
            voltages.0 /= larger_magnitude;
            voltages.1 /= larger_magnitude;
        }

        voltages
    }

    /// Returns the left voltage.
    ///
    /// # Examples
    ///
    /// ```
    /// let voltages = DifferentialVoltages(5.0, 2.0);
    /// assert_eq!(voltages.left(), 5.0);
    /// ```
    #[must_use]
    pub const fn left(&self) -> f64 {
        self.0
    }

    /// Returns the right voltage.
    ///
    /// ```
    /// let voltages = DifferentialVoltages(5.0, 2.0);
    /// assert_eq!(voltages.right(), 2.0);
    /// ```
    #[must_use]
    pub const fn right(&self) -> f64 {
        self.1
    }
}

impl From<(f64, f64)> for DifferentialVoltages {
    fn from(value: (f64, f64)) -> Self {
        Self(value.0, value.1)
    }
}
