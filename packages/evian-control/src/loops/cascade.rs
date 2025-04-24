use core::time::Duration;

use evian_math::Angle;

use super::{
    ControlLoop, Feedback, FeedbackMarker, Feedforward, MotorFeedforward,
    feedforward::{
        ArmFeedforward, ArmFeedforwardSetpoint, ElevatorFeedforward, ElevatorFeedforwardSetpoint,
        MotorFeedforwardSetpoint,
    },
};

/// Combines two control loops together, passing the primary control
/// loop's output into the secondary control loop's setpoint.
pub struct Cascade<P: ControlLoop, S: ControlLoop> {
    pub primary: P,
    pub secondary: S,

    // We incur a slight runtime cost to eat an additional generic arg
    // here and allow for specialized `Self::new` functions involving
    // specific feedforward controllers.
    map_fn: Option<fn(P::Input, P::Output) -> S::Input>,
}

impl<P: Feedback, S: Feedback<Input = P::Output>> Cascade<P, S> {
    pub const fn new(primary: P, secondary: S) -> Self {
        Self {
            secondary,
            primary,
            map_fn: None,
        }
    }
}

impl<P: Feedback, S: Feedback<Input = P::Output>> Cascade<P, S> {
    pub fn update(
        &mut self,
        primary_measurement: P::Input,
        secondary_measurement: S::Input,
        setpoint: P::Input,
        dt: Duration,
    ) -> S::Output {
        self.secondary.update(
            secondary_measurement,
            self.primary.update(primary_measurement, setpoint, dt),
            dt,
        )
    }
}

impl<P: Feedback, S: Feedforward> ControlLoop for Cascade<P, S>
where
    P::Input: Clone,
{
    type Marker = FeedbackMarker;
    type Input = P::Input;
    type Output = S::Output;
}

impl<P: Feedback, S: Feedforward> Cascade<P, S>
where
    P::Input: Clone,
{
    pub const fn map(feedback: P, feedforward: S, f: fn(P::Input, P::Output) -> S::Input) -> Self {
        Self {
            primary: feedback,
            secondary: feedforward,
            map_fn: Some(f as _),
        }
    }
}

impl<P: Feedback<Output = f64>> Cascade<P, MotorFeedforward>
where
    P::Input: Clone,
{
    pub const fn new(feedback: P, feedforward: MotorFeedforward) -> Self {
        Self::map(feedback, feedforward, |_, velocity| {
            MotorFeedforwardSetpoint {
                velocity,
                acceleration: 0.0,
            }
        })
    }
}

impl<P: Feedback<Output = f64>> Cascade<P, ElevatorFeedforward>
where
    P::Input: Clone,
{
    pub const fn new(feedback: P, feedforward: ElevatorFeedforward) -> Self {
        Self::map(feedback, feedforward, |_, velocity| {
            ElevatorFeedforwardSetpoint {
                velocity,
                acceleration: 0.0,
            }
        })
    }
}

impl<P: Feedback<Input = Angle, Output = f64>> Cascade<P, ArmFeedforward>
where
    P::Input: Clone,
{
    pub const fn new(feedback: P, feedforward: ArmFeedforward) -> Self {
        Self {
            primary: feedback,
            secondary: feedforward,
            map_fn: Some(|position, velocity| ArmFeedforwardSetpoint {
                position,
                velocity,
                acceleration: 0.0,
            }),
        }
    }
}

impl<Fb: Feedback, Ff: Feedforward> Feedback for Cascade<Fb, Ff>
where
    Fb::Input: Clone,
{
    fn update(&mut self, measurement: Fb::Input, setpoint: Fb::Input, dt: Duration) -> Ff::Output {
        self.secondary.update(
            // SAFETY: This variant of the struct may only be constructed with Some(fn).
            (unsafe { self.map_fn.unwrap_unchecked() })(
                setpoint.clone(),
                self.primary.update(measurement, setpoint, dt),
            ),
            dt,
        )
    }
}
