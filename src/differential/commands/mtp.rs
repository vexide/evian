use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use vexide::{core::time::Instant, devices::smart::Motor, prelude::Float};

use crate::{
    command::{Command, CommandUpdate}, control::{settler::Settler, MotionController}, differential::Voltages, math::Vec2, tracking::TrackingContext
};

#[derive(Clone, Copy, PartialEq)]
struct MoveToPoint<
    L: MotionController<Input = f64, Output = f64>,
    A: MotionController<Input = f64, Output = f64>,
> {
    initial_cx: Option<TrackingContext>,

    target: Vec2,

    distance_controller: L,
    angle_controller: A,

    settler: Settler,

    prev_timestamp: Instant,
}

impl<
        L: MotionController<Input = f64, Output = f64>,
        A: MotionController<Input = f64, Output = f64>,
    > Command for MoveToPoint<L, A>
{
    type Output = Voltages;

    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output> {
        let dt = self.prev_timestamp.elapsed();

        let local_target = cx.position - self.target;
        let distance_to_point = local_target.length();
        let mut angle_target = local_target.angle();
        let mut angle_error = cx.heading - angle_target;
        
        if angle_error.abs() > FRAC_PI_4 {
            angle_error -= FRAC_PI_2;
            angle_target -= FRAC_PI_2;
        }

        let angular_output = self.angle_controller.update(
            angle_target,
            cx.heading,
            dt,
        );
        let linear_output = self.distance_controller.update(
            distance_to_point,
            0.0,
            dt,
        ) * angle_error.cos();

        self.prev_timestamp = Instant::now();

        CommandUpdate::Update(
            Voltages(
                linear_output + angular_output,
                linear_output - angular_output,
            )
            .normalized(Motor::MAX_VOLTAGE),
        )
    }
}
