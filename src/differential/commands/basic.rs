use core::f64::consts::FRAC_PI_2;

use vexide::{core::time::Instant, devices::smart::Motor};

use crate::{
    command::{Command, CommandUpdate},
    control::Feedback,
    differential::Voltages,
    settler::Settler,
    tracking::TrackingContext,
};

pub struct BasicMotions<
    L: Feedback<Input = f64, Output = f64> + Clone,
    A: Feedback<Input = f64, Output = f64> + Clone,
> {
    pub linear_controller: L,
    pub linear_settler: Settler,
    pub angular_controller: A,
    pub angular_settler: Settler,
}

impl<
        L: Feedback<Input = f64, Output = f64> + Clone,
        A: Feedback<Input = f64, Output = f64> + Clone,
    > BasicMotions<L, A>
{
    pub fn new(
        linear_controller: L,
        angular_controller: A,
        linear_settler: Settler,
        angular_settler: Settler,
    ) -> Self {
        Self {
            linear_controller,
            angular_controller,
            linear_settler,
            angular_settler,
        }
    }

    fn make_command(&self, distance_target: Target, heading_target: Target) -> BasicMotion<L, A> {
        BasicMotion {
            initial_cx: None,

            linear_controller: self.linear_controller.clone(),
            linear_settler: self.linear_settler,
            linear_target: distance_target,

            angular_controller: self.angular_controller.clone(),
            angular_settler: self.angular_settler,
            angular_target: heading_target,

            prev_timestamp: Instant::now(),
        }
    }

    pub fn drive_distance_at_heading(
        &self,
        distance: f64,
        heading: f64,
    ) -> impl Command<Output = Voltages> {
        self.make_command(Target::Relative(distance), Target::Absolute(heading))
    }

    pub fn drive_distance(&self, distance: f64) -> impl Command<Output = Voltages> {
        self.make_command(Target::Relative(distance), Target::Initial)
    }

    pub fn turn_to_heading(&self, heading: f64) -> impl Command<Output = Voltages> {
        self.make_command(Target::Initial, Target::Absolute(heading))
    }

    pub fn turn_by_angle(&self, offset: f64) -> impl Command<Output = Voltages> {
        self.make_command(Target::Initial, Target::Relative(offset))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Target {
    Initial,
    Absolute(f64),
    Relative(f64),
}

#[derive(Clone, Copy, PartialEq)]
struct BasicMotion<L: Feedback<Input = f64, Output = f64>, A: Feedback<Input = f64, Output = f64>> {
    initial_cx: Option<TrackingContext>,

    linear_controller: L,
    linear_settler: Settler,
    linear_target: Target,

    angular_controller: A,
    angular_settler: Settler,
    angular_target: Target,

    prev_timestamp: Instant,
}

impl<L: Feedback<Input = f64, Output = f64>, A: Feedback<Input = f64, Output = f64>> Command
    for BasicMotion<L, A>
{
    type Output = Voltages;

    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output> {
        let dt = self.prev_timestamp.elapsed();

        if self.initial_cx.is_none() {
            self.initial_cx = Some(cx);
        }
        let initial_cx = self.initial_cx.unwrap();

        let target_forward_travel = match self.linear_target {
            Target::Absolute(travel) => travel,
            Target::Relative(distance) => distance + initial_cx.forward_travel,
            Target::Initial => initial_cx.forward_travel,
        };
        let target_heading = match self.angular_target {
            Target::Absolute(heading) => heading,
            Target::Relative(offset) => offset + initial_cx.heading,
            Target::Initial => initial_cx.heading,
        };

        let linear_error = target_forward_travel - cx.forward_travel;
        let angular_error = target_heading % FRAC_PI_2 - cx.heading;

        if self
            .linear_settler
            .is_settled(linear_error, cx.linear_velocity)
            && self
                .angular_settler
                .is_settled(angular_error, cx.angular_velocity)
        {
            return CommandUpdate::Settled;
        }

        let linear_output =
            self.linear_controller
                .update(cx.forward_travel, target_forward_travel, dt);
        let angular_output =
            self.linear_controller
                .update(cx.heading, target_heading % FRAC_PI_2, dt);

        self.prev_timestamp = Instant::now();

        CommandUpdate::Update(
            Voltages(
                linear_output + angular_output,
                linear_output - angular_output,
            )
            .normalized(Motor::MAX_VOLTAGE),
        )
    }
}
