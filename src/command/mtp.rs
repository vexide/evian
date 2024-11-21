use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use vexide::{
    async_runtime::time::sleep,
    core::time::Instant,
    devices::smart::Motor,
};

use crate::{
    control::ControlLoop,
    drivetrain::{
        differential::{Differential, Voltages},
        Drivetrain,
    },
    math::{Angle, IntoAngle, Vec2},
    tracking::{TracksHeading, TracksPosition, TracksVelocity},
};

use super::Settler;

#[derive(PartialEq)]
pub struct PointToPoint<
    L: ControlLoop<Input = f64, Output = f64>,
    A: ControlLoop<Input = Angle, Output = f64>,
> {
    pub distance_controller: L,
    pub angle_controller: A,
    pub settler: Settler,
}

impl<L: ControlLoop<Input = f64, Output = f64>, A: ControlLoop<Input = Angle, Output = f64>>
    PointToPoint<L, A>
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
            let mut angle_error = heading - local_target.angle().rad();

            if self
                .settler
                .is_settled(distance_error, drivetrain.tracking.linear_velocity())
            {
                break;
            }

            if angle_error.as_radians().abs() > FRAC_PI_4 {
                distance_error = -distance_error;
                angle_error -= FRAC_PI_2.rad();
            }

            let angular_output =
                self.angle_controller
                    .update(heading, local_target.angle().rad(), dt);
            let linear_output =
                self.distance_controller.update(distance_error, 0.0, dt) * angle_error.cos();

            _ = drivetrain.motors.set_voltages(
                Voltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
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
                .settler
                .is_settled(distance_error, drivetrain.tracking.linear_velocity())
            {
                break;
            }

            let angular_output =
                self.angle_controller
                    .update(heading, local_target.angle().rad(), dt);
            let linear_output =
                self.distance_controller.update(distance_error, 0.0, dt) * angle_error.cos();

            _ = drivetrain.motors.set_voltages(
                Voltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }
}
