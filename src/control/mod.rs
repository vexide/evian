use core::time::Duration;

pub mod pid;

pub trait Feedback {
    type Input;
    type Output;

    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: Duration,
    ) -> Self::Output;
}

pub trait Feedforward {
    type Input;
    type Output;

    fn update(&mut self, setpoint: Self::Input, dt: Duration) -> Self::Output;
}
