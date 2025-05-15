//! Differential drivetrains.
//!
//! This module provides support for drivetrains configured in a differential (left/right)
//! wheel configuration.
//!
//! # Overview
//!
//! A differential drivetrain (also called a **tank drive** or **skid-steer**) is a robot
//! whose movement is controlled by two independently driven sets of wheels on the left and
//! right sides of its chassis. The system operates by adjusting the speed and direction of
//! the left and right motors, enabling the robot to drive straight or execute turns.

pub mod curvature;

use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

// MARK: Motors

/// A collection of motors mounted in a differential (left/right) configuration.
///
/// A differential drivetrain (also called a **tank drive** or **skid-steer**) is a robot
/// whose movement is controlled by two independently driven sets of wheels on the left and
/// right sides of its chassis. The system operates by adjusting the speed and direction of
/// the left and right motors, enabling the robot to drive straight or execute turns.
///
/// - If both sets of motors move at the same speed, the robot moves straight.
/// - If one set of motors moves faster than the other, the robot will turn.
/// - If the motors on one side move forward while the other side moves backward, the
///   robot will rotate in place.
///
/// Differential drivetrains are *nonholonomic*, meaning they cannot strafe laterally.
pub struct Differential {
    /// Left motors.
    pub left: Rc<RefCell<dyn AsMut<[Motor]>>>,

    /// Right motors.
    pub right: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl Differential {
    /// Creates a new drivetrain with the provided left/right motors.
    ///
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     [
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ],
    ///     [
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ],
    /// );
    /// ```
    pub fn new<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: L,
        right: R,
    ) -> Self {
        Self {
            left: Rc::new(RefCell::new(left)),
            right: Rc::new(RefCell::new(right)),
        }
    }

    /// Creates a new drivetrain with shared ownership of the left/right motors.
    ///
    /// This is similar to [`Differential::new`], except that it allows you to share
    /// your motor collections with other subsystems. A common use-case for this is
    /// in drivetrains that use their own motors as tracking source ("IME-only"
    /// tracking).
    ///
    /// In order to create a drivetrain, the provided motor collections should be
    /// wrapped in an `Rc<RefCell<T>>`.
    ///
    /// # Examples
    ///
    /// ```
    /// let motors = Differential::new(
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
    ///         Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ///     ])),
    ///     Rc::new(RefCell::new([
    ///         Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ///         Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ///     ])),
    /// );
    /// ```
    ///
    /// Alternatively, evian's `drivetrain` module also provides a [`shared_motors`]
    /// macro that simplifies the creation of shared motor arrays
    /// (`Rc<RefCell<[Motor; N]>>`).
    ///
    /// [`shared_motors`]: crate::shared_motors
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
    pub fn from_shared<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: Rc<RefCell<L>>,
        right: Rc<RefCell<R>>,
    ) -> Self {
        Self { left, right }
    }

    /// Sets the voltage of the left and right motors.
    ///
    /// # Errors
    ///
    /// This method will return an error if calling any individual motor's
    /// [`set_voltage`] method fails. If multiple motors fail, then only the
    /// error from the last failed motor will be returned.
    ///
    /// [`set_voltage`]: vexide::devices::smart::motor::Motor::set_voltage
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
    /// motors.set_voltages(Voltages(12.0, 12.0))?;
    /// ```
    pub fn set_voltages(&mut self, voltages: impl Into<Voltages>) -> Result<(), MotorError> {
        let voltages = voltages.into();
        let mut rtn = Ok(());

        for motor in self.left.borrow_mut().as_mut() {
            let result = motor.set_voltage(voltages.left());

            if result.is_err() {
                rtn = result;
            }
        }

        for motor in self.right.borrow_mut().as_mut() {
            let result = motor.set_voltage(voltages.right());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}

// MARK: Voltages

/// Left/Right Motor Voltages
///
/// These voltages are used to control a [`Differential`] motor configuration. They describe
/// the voltages of the respective left and right motors.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Voltages(pub f64, pub f64);

impl Voltages {
    /// Creates a [`Voltages`] instance from a provided linear and angular voltage.
    ///
    /// # Examples
    ///
    /// ```
    /// let voltages = Voltages::from_arcade(5.0, 2.0);
    /// assert_eq!(voltages, Voltages(7.0, 3.0));
    /// ```
    #[must_use]
    pub const fn from_arcade(linear: f64, angular: f64) -> Self {
        Self(linear + angular, linear - angular)
    }

    /// Returns [`Voltages`] that are less than a provided `max` value while
    /// preserving the ratio between the original left and right values.
    ///
    /// If either motor is over a `max_voltage`, both values will be decreased by the
    /// amount that is "oversaturated" to preserve the ratio between left and right power.
    ///
    /// # Examples
    ///
    /// ```
    /// let voltages = Voltages::from_arcade(12.0, 13.0).normalized(12.0);
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
    /// let voltages = Voltages(5.0, 2.0);
    /// assert_eq!(voltages.left(), 5.0);
    /// ```
    #[must_use]
    pub const fn left(&self) -> f64 {
        self.0
    }

    /// Returns the right voltage.
    ///
    /// ```
    /// let voltages = Voltages(5.0, 2.0);
    /// assert_eq!(voltages.right(), 2.0);
    /// ```
    #[must_use]
    pub const fn right(&self) -> f64 {
        self.1
    }
}

impl From<(f64, f64)> for Voltages {
    fn from(value: (f64, f64)) -> Self {
        Self(value.0, value.1)
    }
}
