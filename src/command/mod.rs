//! Robot Motion Commands
//!
//! This module provides traits and types for implementing robot motion commands. A *command*
//! is a general implementation of a motion algorithm, and might perform actions like:
//!
//! - Driving to a target position.
//! - Turning to face a direction.
//! - Following a curve or trajectory.
//! - Maintaining a constant velocity.
//!
//! Commands receive a robot's recorded position and motion state via [`TrackingContext`]
//! and produce an output (typically a set of voltages) that are then fed to motors until
//! the command has achieved its goal (whatever that may be).

pub mod settler;

use crate::tracking::TrackingContext;

/// A robot motion command that produces control outputs and tracks completion.
pub trait Command {
    /// The type of control signal produced by this command. This typically expressed
    /// as a set of voltages that are then used to power motors on a specific type of
    /// drivetrain.
    type Output;

    /// Updates the command's state and returns either a new control output or
    /// indicates the command has settled (completed).
    ///
    /// This method should be called periodically with the latest robot state as
    /// recorded by an implementation of the [`Tracking`](crate::tracking::Tracking) trait.
    ///
    /// Command updates will return [`CommandUpdate::Update`] with a new control output if the
    /// command is still executing, or [`CommandUpdate::Settled`] once the command
    /// has achieved its goal and is now done.
    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output>;
}

/// The output of a [`Command`] update.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CommandUpdate<T> {
    /// The command is still executing and has produced a new control output.
    Update(T),

    /// The command has completed and achieved its goal.
    ///
    /// No further control outputs will be produced by [`Command::update`].
    Settled,
}
