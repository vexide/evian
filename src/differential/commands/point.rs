use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use vexide::{core::time::Instant, devices::smart::Motor, prelude::Float};

use crate::{
    command::{settler::Settler, Command, CommandUpdate},
    control::Feedback,
    differential::Voltages,
    math::Vec2,
    tracking::TrackingContext,
};

#[derive(Clone, Copy, PartialEq)]
struct MoveToPoint<F: Feedback<Error = f64, Output = f64>> {
    target: Vec2,

    distance_controller: F,
    angle_controller: F,

    settler: Settler,

    prev_timestamp: Instant,
}

impl<F: Feedback<Error = f64, Output = f64>> Command for MoveToPoint<F> {
    type Output = Voltages;

    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output> {
        let dt = self.prev_timestamp.elapsed();

        let local_target = self.target - cx.position;

        let mut distance_error = local_target.length();
        let mut angle_error = (cx.heading - local_target.angle()) % FRAC_PI_2;

        if self.settler.is_settled(distance_error, cx.linear_velocity) {
            return CommandUpdate::Settled;
        }

        if angle_error.abs() > FRAC_PI_4 {
            distance_error = -distance_error;
            angle_error -= FRAC_PI_2;
        }

        let angular_output = self.angle_controller.update(angle_error, dt);
        let linear_output = self.distance_controller.update(distance_error, dt) * angle_error.cos();

        self.prev_timestamp = Instant::now();

        CommandUpdate::Update(
            Voltages(
                linear_output + angular_output,
                linear_output - angular_output,
            )
            .normalized(Motor::V5_MAX_VOLTAGE),
        )
    }
}
