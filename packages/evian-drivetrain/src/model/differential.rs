use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

use super::{DrivetrainModel, Tank};

// MARK: Motors

/// Differential drivetrain model.
///
/// A differential drivetrain (also called a **tank drive** or **skid-steer**) is a robot
/// whose movement is controlled by two independently driven sets of wheels on the left and
/// right sides of its chassis. The system operates by adjusting the speed and direction of
/// the left and right motors, enabling the robot to drive straight or execute turns.
///
/// - If both sets of motors move at the same speed, the robot moves straight.
/// - If one set of motors moves faster than the other, the robot will turn.
/// - If the motors on one side move forward while the other side moves backward, the
///   robot will turn in place.
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
}

// MARK: Kinematics

impl DrivetrainModel for Differential {
    type Error = MotorError;
}

impl Tank for Differential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        let mut rtn = Ok(());

        for motor in self.left.borrow_mut().as_mut() {
            let result = motor.set_voltage(left);

            if result.is_err() {
                rtn = result;
            }
        }

        for motor in self.right.borrow_mut().as_mut() {
            let result = motor.set_voltage(right);

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}
