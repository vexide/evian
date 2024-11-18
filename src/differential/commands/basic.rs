use vexide::{
    core::time::Instant,
    devices::smart::Motor,
    prelude::{sleep, SmartDevice},
};

use crate::{
    control::ControlLoop,
    differential::Voltages,
    math::{Angle, Vec2},
    prelude::DifferentialDrivetrain,
    settler::Settler,
};

#[derive(PartialEq)]
pub struct BasicMotion<
    L: ControlLoop<Input = f64, Output = f64>,
    A: ControlLoop<Input = Angle, Output = f64>,
> {
    pub linear_controller: L,
    pub angular_controller: A,
    pub linear_settler: Settler,
    pub angular_settler: Settler,
}

impl<L: ControlLoop<Input = f64, Output = f64>, A: ControlLoop<Input = Angle, Output = f64>>
    BasicMotion<L, A>
{
    pub async fn drive_distance_at_heading(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        target_distance: f64,
        target_heading: Angle,
    ) {
        let initial_forward_travel = drivetrain.tracking_data().forward_travel;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let tracking_data = drivetrain.tracking_data();

            let linear_output = self.linear_controller.update(
                tracking_data.forward_travel,
                target_distance + initial_forward_travel,
                dt,
            );
            let angular_output =
                self.angular_controller
                    .update(tracking_data.heading, target_heading, dt);

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
            .await;
    }

    pub async fn turn_to_heading(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        heading: Angle,
    ) {
        self.drive_distance_at_heading(drivetrain, 0.0, heading)
            .await;
    }

    pub async fn turn_to_point(
        &mut self,
        drivetrain: &mut DifferentialDrivetrain,
        point: impl Into<Vec2<f64>>,
    ) {
        let point = point.into();
        let initial_forward_travel = drivetrain.tracking_data().forward_travel;
        let mut prev_time = Instant::now();

        loop {
            sleep(Motor::UPDATE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let tracking_data = drivetrain.tracking_data();

            let linear_output = self.linear_controller.update(
                tracking_data.forward_travel,
                initial_forward_travel,
                dt,
            );
            let angular_output = self.angular_controller.update(
                tracking_data.heading,
                Angle::from_radians((point - tracking_data.position).angle()),
                dt,
            );

            _ = drivetrain.set_voltages(
                Voltages::from_arcade(linear_output, angular_output)
                    .normalized(Motor::V5_MAX_VOLTAGE),
            );

            prev_time = Instant::now();
        }
    }
}
