use num_traits::real::Real;
use vexide::devices::smart::Motor;

use crate::{
    commands::Command, controller::MotionController, drivetrain::Voltages, math,
    tracking::TrackingContext,
};

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct BasicMotion<
    D: MotionController<Input = f64, Output = f64>,
    T: MotionController<Input = f64, Output = f64>,
> {
    distance: f64,
    angle: f64,

    drive_tolerance: f64,
    turn_tolerance: f64,

    drive_error: f64,
    turn_error: f64,

    drive_controller: D,
    turn_controller: T,
}

impl<
        D: MotionController<Input = f64, Output = f64>,
        T: MotionController<Input = f64, Output = f64>,
    > Command for BasicMotion<D, T>
{
    type Output = Voltages;

    fn update(&mut self, ctx: TrackingContext) -> Self::Output {
        self.drive_error = self.distance - ctx.forward_travel;
        self.turn_error = math::normalize_angle(self.angle - ctx.heading);

        let drive_output = self.drive_controller.update(self.drive_error);
        let turn_output = self.drive_controller.update(self.turn_error);

        Voltages(drive_output + turn_output, drive_output - turn_output)
            .normalized(Motor::MAX_VOLTAGE)
    }

    fn is_settled(&self) -> bool {
        self.drive_error.abs() < self.drive_tolerance && self.turn_error.abs() < self.turn_tolerance
    }

    fn cancel(&mut self) {}
}
