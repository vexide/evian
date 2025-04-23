//! Robot drivetrain configurations.
//!
//! This crate provides types for describing and modeling different mobile robot drivetrain
//! configurations. A *drivetrain* in evian is the combination of hardware components (e.g. motors,
//! wheels, and sensors) that enables a robot to both *move* and *track its motion*. This importantly
//! means that drivetrains are a collection of **both** motors and sensors.
//!
//! At the heart of this crate is the [`Drivetrain`] struct, which bundles together some motors and a
//! *tracking system* — a system that measures something about the drivetrain as it moves around.
//! The [`Drivetrain`] type could represent many different types of robot drivetrains depending on how
//! the motor and tracking logic is implemented.
//!
//! # Supported Configurations
//!
//! At the moment, this crate currently provides built-in support for [differential drivetrains],
//! however the [`Drivetrain`] struct could in theory be configured to accept any arrangement of motors
//! with your own custom type if you require something else.
//!
//! [differential drivetrains]: crate::differential

#![no_std]

extern crate alloc;

pub mod differential;

/// A mobile robot drivetrain capable of measuring data about itself.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Drivetrain<M, T> {
    /// Motor collection.
    pub motors: M,

    /// Tracking system.
    pub tracking: T,
}

impl<M, T> Drivetrain<M, T> {
    /// Creates a new drivetrain from a collection of motors and a tracking system.
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
