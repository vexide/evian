use core::time::Duration;

pub mod pid;
pub mod settler;

pub trait MotionController {
    type Input;
    type Output;

    fn update(
        &mut self,
        process_value: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}
