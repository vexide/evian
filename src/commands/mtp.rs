use vexide::devices::{controller::Controller, controller::ControllerError, smart::Motor};

use crate::drivetrain::Voltages;

pub struct MoveToPoint<D: MotionController, T: MotionController, S: SettleCondition> {
	drive_controller: PIDController,
	turn_controller: PIDController,
	drive_error: f64,
	turn_error: f64,
}
