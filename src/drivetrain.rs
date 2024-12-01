use core::cell::RefCell;

use alloc::{rc::Rc, vec::Vec};
use vexide::prelude::Motor;

/// A mobile robot capable of measuring data about itself.
#[derive(Default, Debug, Eq, PartialEq, Hash)]
pub struct Drivetrain<M, T> {
    pub motors: M,
    pub tracking: T,
}

impl<M, T> Drivetrain<M, T> {
    pub const fn new(motors: M, tracking: T) -> Self {
        Self { motors, tracking }
    }
}

// Internal alias so I don't have to type this shit out a million times.
pub type SharedMotors = Rc<RefCell<Vec<Motor>>>;

/// A macro that creates a set of motors for a [`DifferentialDrivetrain`].
///
/// This macro simplifies the creation of a [`DriveMotors`] collection, which is a sharable, threadsafe
/// wrapper around vexide's non-copyable [`Motor`](vexide::devices::smart::motor::Motor) struct.
///
/// # Examples
///
/// ```
/// let motors = drive_motors![motor1, motor2, motor3];
/// ```
#[macro_export]
macro_rules! shared_motors {
    ( $( $item:expr ),* $(,)?) => {
        {
            use ::core::cell::RefCell;
            use ::alloc::{rc::Rc, vec::Vec};

            let mut temp_vec: Vec<Motor> = Vec::new();

            $(
                temp_vec.push($item);
            )*

            Rc::new(RefCell::new(temp_vec))
        }
    };
}
pub use shared_motors;
