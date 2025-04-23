use core::{ops::Add, time::Duration};

use super::{
    ControlLoop, Feedback, Feedforward, MotorFeedforward, feedforward::MotorFeedforwardState,
};

/// A control loop that adds the output of two other control loops together.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Combine<L, R>
where
    L: ControlLoop,
    L::Input: Clone,
    R: ControlLoop,
{
    /// Left-hand-side of the addition operation.
    pub left: L,

    /// Right-hand-side of the addition operation.
    pub right: R,
}

impl<L, R> Combine<L, R>
where
    L: ControlLoop,
    L::Input: Clone,
    R: ControlLoop,
{
    /// Combines two [`ControlLoop`]s together into a single
    /// [`ControlLoop`] that adds their outputs together.
    pub const fn new(left: L, right: R) -> Self {
        Self { left, right }
    }
}

// MARK: Feedback + Feedback

impl<L, R> ControlLoop for Combine<L, R>
where
    L: Feedback,
    L::Input: Clone,
    L::Output: Add<Output = L::Output>,
    R: Feedback<Input = L::Input, Output = L::Output>,
{
    type Input = L::Input;
    type Output = L::Output;
}

impl<L, R> Feedback for Combine<L, R>
where
    L: Feedback,
    L::Input: Clone,
    L::Output: Add<Output = L::Output>,
    R: Feedback<Input = L::Input, Output = L::Output>,
{
    fn update(
        &mut self,
        measurement: Self::Input,
        setpoint: Self::Input,
        dt: core::time::Duration,
    ) -> Self::Output {
        let left_signal = self.left.update(measurement.clone(), setpoint.clone(), dt);
        let right_signal = self.right.update(measurement, setpoint, dt);

        left_signal + right_signal
    }
}

// MARK: Feedback + DcMotorFeedforward

impl<L> ControlLoop for Combine<L, MotorFeedforward>
where
    L: Feedback<Input = f64>,
    L::Output: Add<f64, Output = f64>,
{
    type Input = f64;
    type Output = f64;
}

impl<L> Feedback for Combine<L, MotorFeedforward>
where
    L: Feedback<Input = f64>,
    L::Output: Add<f64, Output = f64>,
{
    fn update(&mut self, measurement: f64, setpoint: Self::Input, dt: core::time::Duration) -> f64 {
        let left_signal = self.left.update(measurement, setpoint, dt);
        let right_signal = self.right.update(
            MotorFeedforwardState {
                velocity: setpoint,
                acceleration: 0.0,
            },
            dt,
        );

        left_signal + right_signal
    }
}

// MARK: DcMotorFeedforward + Feedback

impl<R: Feedback<Input = f64, Output = f64>> ControlLoop for Combine<MotorFeedforward, R> {
    type Input = f64;
    type Output = f64;
}

impl<R: Feedback<Input = f64, Output = f64>> Feedback for Combine<MotorFeedforward, R> {
    fn update(&mut self, measurement: f64, setpoint: f64, dt: Duration) -> f64 {
        let left_signal = self.left.update(
            MotorFeedforwardState {
                velocity: setpoint,
                acceleration: 0.0,
            },
            dt,
        );
        let right_signal = self.right.update(measurement, setpoint, dt);

        left_signal + right_signal
    }
}
