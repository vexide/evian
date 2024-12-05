use vexide::{
    core::time::Instant,
    devices::smart::Motor,
    prelude::{sleep, SmartDevice},
};

use crate::{
    control::{ControlLoop, Tolerances}, differential::{Differential, DifferentialVoltages}, drivetrain::Drivetrain, math::{Angle, IntoAngle, Vec2}, prelude::TracksVelocity, tracking::{TracksForwardTravel, TracksHeading, TracksPosition}
};

/// Basic Driving & Turning Motion
///
/// This struct provides motion algorithms for basic control of a differential drivetrain. It
/// includes straight distance driving and turning (both to a angle through [`turn_to_heading`](BasicMotion::turn_to_heading)
/// and to points through [`turn_to_point`](BasicMotion::turn_to_point)). This is acomplished through two feedback
/// control loops (typically PID controllers) for controlling the robot's desired heading and distance traveled.
#[derive(PartialEq)]
pub struct BasicMotion<
    L: ControlLoop<Input = f64, Output = f64>,
    A: ControlLoop<Input = Angle, Output = f64>,
> {
    /// Linear (forward driving) feedback controller
    pub linear_controller: L,

    /// Angular (turning) feedback controller
    pub angular_controller: A,

    /// Linear settling conditions
    pub linear_tolerances: Tolerances,

    /// Angular settling conditions
    pub angular_tolerances: Tolerances,
}

impl<L: ControlLoop<Input = f64, Output = f64>, A: ControlLoop<Input = Angle, Output = f64>>
    BasicMotion<L, A>
{
    pub async fn drive_distance_at_heading<T: TracksForwardTravel + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        target_distance: f64,
        target_heading: Angle,
    ) {
        let initial_forward_travel = drivetrain.tracking.forward_travel();
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let forward_travel = drivetrain.tracking.forward_travel();
            let heading = drivetrain.tracking.heading();

            let linear_output = self.linear_controller.update(
                forward_travel,
                target_distance + initial_forward_travel,
                dt,
            );
            let angular_output = self.angular_controller.update(heading, target_heading, dt);

            _ = drivetrain.motors.set_voltages(
                DifferentialVoltages(
                    linear_output + angular_output,
                    linear_output - angular_output,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }

    pub async fn drive_distance<T: TracksForwardTravel + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        distance: f64,
    ) {
        self.drive_distance_at_heading(drivetrain, distance, drivetrain.tracking.heading())
            .await;
    }

    pub async fn turn_to_heading<T: TracksForwardTravel + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        heading: Angle,
    ) {
        self.drive_distance_at_heading(drivetrain, 0.0, heading)
            .await;
    }

    pub async fn turn_to_point<T: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
    ) {
        let point = point.into();
        let initial_forward_travel = drivetrain.tracking.forward_travel();
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let forward_travel = drivetrain.tracking.forward_travel();
            let position = drivetrain.tracking.position();
            let heading = drivetrain.tracking.heading();

            // TODO: settle

            let linear_output =
                self.linear_controller
                    .update(forward_travel, initial_forward_travel, dt);
            let angular_output =
                self.angular_controller
                    .update(heading, (point - position).angle().rad(), dt);

            _ = drivetrain.motors.set_voltages(
                DifferentialVoltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }
}
