use vexide::devices::{controller::Controller, controller::ControllerError, smart::Motor};

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
        let (left_voltage, right_voltage) = match layout {
            JoystickLayout::Tank => (
                Motor::MAX_VOLTAGE * self.left_stick.y()? as f64,
                Motor::MAX_VOLTAGE * self.right_stick.y()? as f64,
            ),
            _ => todo!(),
        };

        Ok(Voltages(left_voltage, right_voltage))
    }
}
