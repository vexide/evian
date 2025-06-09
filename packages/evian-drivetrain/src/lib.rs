//! Robot drivetrains and models.
//!
//! This crate provides types for representing and modeling different mobile robot drivetrain
//! configurations. A *drivetrain* in evian is the combination of a *model* which describes how the
//! is able to move and a *tracking* system, which describes how the robot can track its motion.
//!
//! At the heart of this crate is the [`Drivetrain`] struct, which bundles together a model and a
//! tracking system. The [`Drivetrain`] type can enacapsulate many different types of robot
//! drivetrains depending on how its model and tracking logic is implemented.

#![no_std]

extern crate alloc;

pub mod model;

use evian_tracking::Tracking;

use model::DrivetrainModel;

/// A mobile robot drivetrain capable of measuring data about itself.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Drivetrain<M: DrivetrainModel, T: Tracking> {
    /// Motor collection.
    pub model: M,

    /// Tracking system.
    pub tracking: T,
}

impl<M: DrivetrainModel, T: Tracking> Drivetrain<M, T> {
    /// Creates a new drivetrain from a collection of motors and a tracking system.
    pub const fn new(model: M, tracking: T) -> Self {
        Self { model, tracking }
    }
}
