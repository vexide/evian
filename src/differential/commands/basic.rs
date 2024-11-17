use vexide::{
    core::time::Instant,
    devices::smart::Motor,
    prelude::{sleep, SmartDevice},
};

use crate::{
    control::Feedback, differential::Voltages, math::Vec2, prelude::DifferentialDrivetrain,
    settler::Settler,
};

use super::normalize_radians;

#[derive(PartialEq)]
pub struct BasicMotion<
    L: Feedback<Error = f64, Output = f64>,
    A: Feedback<Error = f64, Output = f64>,
> {
    pub linear_controller: L,
    pub angular_controller: A,
    pub linear_settler: Settler,
    pub angular_settler: Settler,
}

impl<L: Feedback<Error = f64, Output = f64>, A: Feedback<Error = f64, Output = f64>>
    BasicMotion<L, A>
{
    pub async fn drive_distance_at_heading(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        target_distance: f64,
        target_heading: f64,
    ) {
        let initial_forward_travel = drivetrain.tracking_data().forward_travel;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let tracking_data = drivetrain.tracking_data();

            let linear_error =
                (target_distance + initial_forward_travel) - tracking_data.forward_travel;
            let angular_error = normalize_radians(tracking_data.heading - target_heading);

            let linear_output = self.linear_controller.update(linear_error, dt);
            let angular_output = self.angular_controller.update(angular_error, dt);

            _ = drivetrain.set_voltages(
                Voltages(
                    linear_output + angular_output,
                    linear_output - angular_output,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }

    pub async fn drive_distance(&mut self, drivetrain: &mut DifferentialDrivetrain, distance: f64) {
        self.drive_distance_at_heading(drivetrain, distance, drivetrain.tracking_data().heading)
            .await
    }

    pub async fn turn_to_heading(&mut self, drivetrain: &mut DifferentialDrivetrain, heading: f64) {
        self.drive_distance_at_heading(drivetrain, 0.0, heading)
            .await
    }

    pub async fn turn_to_point(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        point: impl Into<Vec2>,
    ) {
        let point = point.into();
        let initial_forward_travel = drivetrain.tracking_data().forward_travel;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let tracking_data = drivetrain.tracking_data();

            let linear_error = initial_forward_travel - tracking_data.forward_travel;
            let angular_error =
                normalize_radians(tracking_data.heading - (point - tracking_data.position).angle());

            let linear_output = self.linear_controller.update(linear_error, dt);
            let angular_output = self.angular_controller.update(angular_error, dt);

            _ = drivetrain.set_voltages(
                Voltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }
}
