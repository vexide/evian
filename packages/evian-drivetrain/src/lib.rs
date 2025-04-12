#![no_std]

extern crate alloc;

pub mod differential;

/// A mobile robot drivetrain capable of measuring data about itself.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Drivetrain<M, T> {
    pub motors: M,
    pub tracking: T,
}

impl<M, T> Drivetrain<M, T> {
    pub const fn new(motors: M, tracking: T) -> Self {
        Self { motors, tracking }
    }
}

/// Creates a shared motor array.
///
/// This macro simplifies the creation of an `Rc<RefCell<[Motor; N]>>` array, which is a shareable
/// wrapper around vexide's non-copyable [`Motor`](vexide::devices::smart::motor::Motor) struct.
///
/// # Examples
///
/// ```
/// let motors = shared_motors![motor1, motor2, motor3];
/// ```
#[macro_export]
macro_rules! shared_motors {
    ( $( $item:expr ),* $(,)?) => {
        {
            use ::core::cell::RefCell;
            use ::alloc::{rc::Rc, vec::Vec};

            Rc::new(RefCell::new([$($item,)*]))
        }
    };
}
