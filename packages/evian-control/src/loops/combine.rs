use core::{ops::Add, time::Duration};

use super::{
    ControlLoop, Feedback, Feedforward, MotorFeedforward, feedforward::MotorFeedforwardState,
};

/// A control loop that adds the output of two other control loops together.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Combine<L, R>
where
    L: ControlLoop,
    L::State: Clone,
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
    L::State: Clone,
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
    L::State: Clone,
    L::Signal: Add<Output = L::Signal>,
    R: Feedback<State = L::State, Signal = L::Signal>,
{
    type State = L::State;
    type Signal = L::Signal;
}

impl<L, R> Feedback for Combine<L, R>
where
    L: Feedback,
    L::State: Clone,
    L::Signal: Add<Output = L::Signal>,
    R: Feedback<State = L::State, Signal = L::Signal>,
{
    fn update(
        &mut self,
        measurement: Self::State,
        setpoint: Self::State,
        dt: core::time::Duration,
    ) -> Self::Signal {
        let left_signal = self.left.update(measurement.clone(), setpoint.clone(), dt);
        let right_signal = self.right.update(measurement, setpoint, dt);

        left_signal + right_signal
    }
}

// MARK: Feedback + DcMotorFeedforward

impl<L> ControlLoop for Combine<L, MotorFeedforward>
where
    L: Feedback<State = f64>,
    L::Signal: Add<f64, Output = f64>,
{
    type State = f64;
    type Signal = f64;
}

impl<L> Feedback for Combine<L, MotorFeedforward>
where
    L: Feedback<State = f64>,
    L::Signal: Add<f64, Output = f64>,
{
    fn update(&mut self, measurement: f64, setpoint: Self::State, dt: core::time::Duration) -> f64 {
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

impl<R: Feedback<State = f64, Signal = f64>> ControlLoop for Combine<MotorFeedforward, R> {
    type State = f64;
    type Signal = f64;
}

impl<R: Feedback<State = f64, Signal = f64>> Feedback for Combine<MotorFeedforward, R> {
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
