//! Feedback-driven driving and turning.

use core::time::Duration;

use evian_control::{Tolerances, loops::Feedback};
use evian_drivetrain::{
    Drivetrain,
    model::{Arcade, Differential},
};
use evian_math::{Angle, Vec2};
use evian_tracking::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};

mod distance_at_heading;
mod turn_to_point;

pub use distance_at_heading::DriveDistanceAtHeadingFuture;
pub use turn_to_point::TurnToPointFuture;

/// Feedback-driven driving and turning.
#[derive(PartialEq)]
pub struct Basic<L, A>
where
    L: Feedback<Input = f64, Output = f64> + Unpin + Clone,
    A: Feedback<Input = Angle, Output = f64> + Unpin + Clone,
{
    /// Linear (forward driving) feedback controller.
    pub linear_controller: L,

    /// Angular (turning) feedback controller.
    pub angular_controller: A,

    /// Linear settling conditions.
    pub linear_tolerances: Tolerances,

    /// Angular settling conditions.
    pub angular_tolerances: Tolerances,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<L, A> Basic<L, A>
where
    L: Feedback<Input = f64, Output = f64> + Unpin + Clone,
    A: Feedback<Input = Angle, Output = f64> + Unpin + Clone,
{
    /// Moves the robot forwards by a given distance (measured in wheel units) while
    /// turning to face a heading.
    ///
    /// Negative `target_distance` values will move the robot backwards.
    pub fn drive_distance_at_heading<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        target_distance: f64,
        target_heading: Angle,
    ) -> DriveDistanceAtHeadingFuture<'a, M, L, A, T> {
        DriveDistanceAtHeadingFuture {
            target_distance,
            target_heading,
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }

    /// Moves the robot forwards by a given distance (measured in wheel units).
    ///
    /// Negative `distance` values will move the robot backwards.
    pub fn drive_distance<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        distance: f64,
    ) -> DriveDistanceAtHeadingFuture<'a, M, L, A, T> {
        self.drive_distance_at_heading(drivetrain, distance, drivetrain.tracking.heading())
    }

    /// Turns the robot in place to face a heading.
    pub fn turn_to_heading<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        heading: Angle,
    ) -> DriveDistanceAtHeadingFuture<'a, M, L, A, T> {
        self.drive_distance_at_heading(drivetrain, 0.0, heading)
    }

    /// Turns the robot in place to face a 2D point.
    pub fn turn_to_point<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        point: impl Into<Vec2<f64>>,
    ) -> TurnToPointFuture<'a, M, L, A, T> {
        TurnToPointFuture {
            point: point.into(),
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }
}
