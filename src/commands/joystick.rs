use pros::devices::{controller::ControllerError, smart::Motor, Controller};

use crate::drivetrain::Voltages;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JoystickLayout {
    Tank,
    SplitArcade,
    LeftArcade,
    RightArcade,
}

pub trait JoystickCommands {
	fn command(&self, layout: JoystickLayout) -> Result<Voltages, ControllerError>;
}

impl JoystickCommands for Controller {
	fn command(&self, layout: JoystickLayout) -> Result<Voltages, ControllerError> {
		let state = self.state()?;
		let left_stick = state.joysticks.left;
		let right_stick = state.joysticks.right;
		
        let (left_voltage, right_voltage) = match layout {
            JoystickLayout::Tank => (
                Motor::MAX_VOLTAGE * left_stick.y as f64,
                Motor::MAX_VOLTAGE * right_stick.y as f64,
            ),
            _ => todo!(),
        };

        Ok(Voltages(left_voltage, right_voltage))
    }
}