use core::time::Duration;

use super::{
    ControlLoop, Feedback, Feedforward, MotorFeedforward, feedforward::MotorFeedforwardState,
};

/// Combines a feedback controller with a feedforward controller by
/// passing the feedback controller's signal into the feedforward
/// controller's setpoint.
pub struct Cascade<Fb: Feedback, Ff: Feedforward> {
    /// Feedback controller.
    pub feedback: Fb,

    /// Feedforward controller.
    pub feedforward: Ff,
}

impl<Fb: Feedback<Signal = f64>, Ff: Feedforward> Cascade<Fb, Ff> {
    /// Creates a new cascading controller from a feedback and a feedforward
    /// controller.
    pub const fn new(feedback: Fb, feedforward: Ff) -> Self {
        Self {
            feedback,
            feedforward,
        }
    }
}

// MARK: MotorFeedforward

impl<Fb: Feedback<Signal = f64>> ControlLoop for Cascade<Fb, MotorFeedforward> {
    type State = Fb::State;
    type Signal = f64;
}

impl<Fb: Feedback<Signal = f64>> Feedback for Cascade<Fb, MotorFeedforward> {
    fn update(&mut self, measurement: Self::State, setpoint: Self::State, dt: Duration) -> f64 {
        let feedback = self.feedback.update(measurement, setpoint, dt);
        self.feedforward.update(
            MotorFeedforwardState {
                velocity: feedback,
                acceleration: 0.0,
            },
            dt,
        )
    }
}
