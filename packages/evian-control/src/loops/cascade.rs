use core::time::Duration;

use super::{
    ControlLoop, Feedback, Feedforward, MotorFeedforward, feedforward::MotorFeedforwardState,
};

/// Combines a feedback controller with a feedforward controller by
/// passing the feedback controller's signal into the feedforward
/// controller's setpoint.
pub struct Cascade<P: ControlLoop, S: ControlLoop> {
    pub primary: P,
    pub secondary: S,
}

impl<P: Feedback<Output = f64>, S: Feedforward> Cascade<P, S> {
    /// Creates a new cascading controller from a feedback and a feedforward
    /// controller.
    pub const fn new(primary: P, secondary: S) -> Self {
        Self { primary, secondary }
    }
}

impl<P: Feedback<Input = f64>, S: Feedback<Input = P::Output, Output = f64>> Cascade<P, S> {
    fn update(
        &mut self,
        primary_measurement: P::Input,
        secondary_measurement: S::Input,
        setpoint: P::Input,
        dt: Duration,
    ) -> f64 {
        self.secondary.update(
            secondary_measurement,
            self.primary.update(primary_measurement, setpoint, dt),
            dt,
        )
    }
}

// MARK: Feedback -> FeedForward

impl<P: Feedback<Output = f64>> ControlLoop for Cascade<P, MotorFeedforward> {
    type Input = P::Input;
    type Output = f64;
}

impl<P: Feedback<Output = f64>> Feedback for Cascade<P, MotorFeedforward> {
    fn update(&mut self, measurement: Self::Input, setpoint: Self::Input, dt: Duration) -> f64 {
        let feedback = self.primary.update(measurement, setpoint, dt);
        self.secondary.update(
            MotorFeedforwardState {
                velocity: feedback,
                acceleration: 0.0,
            },
            dt,
        )
    }
}
