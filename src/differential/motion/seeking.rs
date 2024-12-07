use core::f64::consts::{FRAC_PI_2, PI};

use vexide::{
    async_runtime::time::sleep,
    core::time::Instant,
    devices::smart::Motor,
};

use crate::{
    control::{ControlLoop, Tolerances},
    differential::{Differential, DifferentialVoltages},
    drivetrain::Drivetrain,
    math::{Angle, IntoAngle, Vec2},
    tracking::{TracksHeading, TracksPosition, TracksVelocity},
};

/// Point-to-Point Feedback Seeking
///
/// This struct provides implementations of adaptive feedback seeking algorithms, which
/// utilize two feedback controllers (one for straight driving and one for turning) to
/// reach a desired point. This is most commonly done using two PID controllers.
///
/// Seeking motions include:
/// - [`move_to_point`](Seeking::move_to_point), which moves the drivetrain to a desired point.
/// - [`boomerang`](Seeking::move_to_point), which moves the drivetrain to a desired pose (including heading).
#[derive(PartialEq)]
pub struct Seeking<
    L: ControlLoop<Input = f64, Output = f64>,
    A: ControlLoop<Input = Angle, Output = f64>,
> {
    pub distance_controller: L,
    pub angle_controller: A,
    pub tolerances: Tolerances,
}

impl<L: ControlLoop<Input = f64, Output = f64>, A: ControlLoop<Input = Angle, Output = f64>>
    Seeking<L, A>
{
    pub async fn move_to_point<T: TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
    ) {
        let point = point.into();
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let position = drivetrain.tracking.position();
            let heading = drivetrain.tracking.heading();

            let local_target = point - position;

            let mut distance_error = local_target.length();
            let mut angle_error = (heading - local_target.angle().rad()).wrapped();

            if angle_error.as_radians().abs() > FRAC_PI_2 {
                distance_error *= -1.0;
                angle_error = (PI.rad() - angle_error).wrapped();
            }

            if self
                .tolerances
                .check(distance_error, drivetrain.tracking.linear_velocity())
            {
                break;
            }

            let angular_output = self.angle_controller.update(-angle_error, Angle::ZERO, dt);
            let linear_output =
                self.distance_controller.update(-distance_error, 0.0, dt) * angle_error.cos();

            _ = drivetrain.motors.set_voltages(
                DifferentialVoltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }

        _ = drivetrain.motors.set_voltages((0.0, 0.0));
    }

    pub async fn boomerang<T: TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
        heading: Angle,
        d_lead: f64,
    ) {
        let point = point.into();
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let position = drivetrain.tracking.position();

            let carrot =
                point - Vec2::from_polar(position.distance(point), heading.as_radians()) * d_lead;

            let local_target = carrot - position;

            let distance_error = local_target.length();
            let angle_error = drivetrain.tracking.heading() - local_target.angle().rad();

            if self
                .tolerances
                .check(distance_error, drivetrain.tracking.linear_velocity())
            {
                break;
            }

            let angular_output =
                self.angle_controller
                    .update(heading, local_target.angle().rad(), dt);
            let linear_output =
                self.distance_controller.update(distance_error, 0.0, dt) * angle_error.cos();

            _ = drivetrain.motors.set_voltages(
                DifferentialVoltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }

        _ = drivetrain.motors.set_voltages((0.0, 0.0));
    }
}
