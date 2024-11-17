use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use vexide::{
    async_runtime::time::sleep,
    core::{float::Float, time::Instant},
    devices::smart::Motor,
};

use crate::{
    control::Feedback, differential::Voltages, math::Vec2, prelude::DifferentialDrivetrain,
    settler::Settler,
};

use super::normalize_radians;

#[derive(PartialEq)]
pub struct MoveToPoint<
    L: Feedback<Error = f64, Output = f64>,
    A: Feedback<Error = f64, Output = f64>,
> {
    pub distance_controller: L,
    pub angle_controller: A,
    pub settler: Settler,
}

impl<L: Feedback<Error = f64, Output = f64>, A: Feedback<Error = f64, Output = f64>>
    MoveToPoint<L, A>
{
    pub async fn move_to_point(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        point: impl Into<Vec2>,
    ) {
        let point = point.into();
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let tracking_data = drivetrain.tracking_data();

            let local_target = point - tracking_data.position;

            let mut distance_error = local_target.length();
            let mut angle_error = normalize_radians(tracking_data.heading - local_target.angle());

            if self
                .settler
                .is_settled(distance_error, tracking_data.linear_velocity)
            {
                break;
            }

            if angle_error.abs() > FRAC_PI_4 {
                distance_error = -distance_error;
                angle_error = normalize_radians(angle_error - FRAC_PI_2);
            }

            let angular_output = self.angle_controller.update(angle_error, dt);
            let linear_output =
                self.distance_controller.update(distance_error, dt) * angle_error.cos();

            _ = drivetrain.set_voltages(
                Voltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }
}
