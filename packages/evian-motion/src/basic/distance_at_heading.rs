use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
    time::Duration,
};

use vexide::time::{Instant, Sleep, sleep};

use evian_control::{
    Tolerances,
    loops::{AngularPid, Feedback, Pid},
};
use evian_drivetrain::{Drivetrain, model::Arcade};
use evian_math::Angle;
use evian_tracking::{TracksForwardTravel, TracksHeading, TracksVelocity};

pub(crate) struct State {
    sleep: Sleep,
    initial_forward_travel: f64,
    start_time: Instant,
    prev_time: Instant,
    linear_settled: bool,
    angular_settled: bool,
}

/// Drives the robot forward or backwards for a distance at a given heading.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct DriveDistanceAtHeadingFuture<'a, M, L, A, T>
where
    M: Arcade,
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
{
    pub(crate) target_distance: f64,
    pub(crate) target_heading: Angle,
    pub(crate) timeout: Option<Duration>,
    pub(crate) linear_tolerances: Tolerances,
    pub(crate) angular_tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
    pub(crate) drivetrain: &'a mut Drivetrain<M, T>,

    /// Internal future state ("local variables").
    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<M, L, A, T> Future for DriveDistanceAtHeadingFuture<'_, M, L, A, T>
where
    M: Arcade,
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();
            State {
                sleep: sleep(Duration::from_millis(5)),
                initial_forward_travel: this.drivetrain.tracking.forward_travel(),
                start_time: now,
                prev_time: now,
                linear_settled: false,
                angular_settled: false,
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = state.prev_time.elapsed();

        let forward_travel = this.drivetrain.tracking.forward_travel();
        let heading = this.drivetrain.tracking.heading();

        let linear_error = (this.target_distance + state.initial_forward_travel) - forward_travel;
        let angular_error = (this.target_heading - heading).wrapped();

        if this
            .linear_tolerances
            .check(linear_error, this.drivetrain.tracking.linear_velocity())
        {
            state.linear_settled = true;
        }
        if this.angular_tolerances.check(
            angular_error.as_radians(),
            this.drivetrain.tracking.angular_velocity(),
        ) {
            state.angular_settled = true;
        }

        if (state.linear_settled && state.angular_settled)
            || this
                .timeout
                .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            drop(this.drivetrain.model.drive_arcade(0.0, 0.0));
            return Poll::Ready(());
        }

        let linear_output = this.linear_controller.update(
            forward_travel,
            this.target_distance + state.initial_forward_travel,
            dt,
        );
        let angular_output = this
            .angular_controller
            .update(heading, this.target_heading, dt);

        drop(this
            .drivetrain
            .model
            .drive_arcade(linear_output, angular_output));

        state.sleep = sleep(Duration::from_millis(5));
        state.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Generic Modifiers

impl<M, L, A, T> DriveDistanceAtHeadingFuture<'_, M, L, A, T>
where
    M: Arcade,
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
{
    /// Modifies this motion's linear feedback controller.
    pub fn with_linear_controller(&mut self, controller: L) -> &mut Self {
        self.linear_controller = controller;
        self
    }

    /// Modifies this motion's angular feedback controller.
    pub fn with_angular_controller(&mut self, controller: A) -> &mut Self {
        self.angular_controller = controller;
        self
    }

    /// Modifies this motion's timeout duration.
    pub const fn with_timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = Some(timeout);
        self
    }

    /// Removes this motion's timeout duration.
    pub const fn without_timeout(&mut self) -> &mut Self {
        self.timeout = None;
        self
    }

    /// Modifies this motion's linear tolerances.
    pub const fn with_linear_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.linear_tolerances = tolerances;
        self
    }

    /// Modifies this motion's linear error tolerance.
    pub const fn with_linear_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's linear error tolerance.
    pub const fn without_linear_error_tolerance(&mut self) -> &mut Self {
        self.linear_tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's linear velocity tolerance.
    pub const fn with_linear_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's linear velocity tolerance.
    pub const fn without_linear_velocity_tolerance(&mut self) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's linear tolerance duration.
    pub const fn with_linear_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.linear_tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's linear tolerance duration.
    pub const fn without_linear_tolerance_duration(&mut self) -> &mut Self {
        self.linear_tolerances.duration = None;
        self
    }

    /// Removes this motion's linear and angular tolerance durations.
    pub const fn without_tolerance_duration(&mut self) -> &mut Self {
        self.linear_tolerances.duration = None;
        self.angular_tolerances.duration = None;
        self
    }

    /// Modifies this motion's angular tolerances.
    pub const fn with_angular_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.angular_tolerances = tolerances;
        self
    }

    /// Modifies this motion's angular error tolerance.
    pub const fn with_angular_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's angular error tolerance.
    pub const fn without_angular_error_tolerance(&mut self) -> &mut Self {
        self.angular_tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's angular velocity tolerance.
    pub const fn with_angular_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's angular velocity tolerance.
    pub const fn without_angular_velocity_tolerance(&mut self) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's angular tolerance duration.
    pub const fn with_angular_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.angular_tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's angular tolerance duration.
    pub const fn without_angular_tolerance_duration(&mut self) -> &mut Self {
        self.angular_tolerances.duration = None;
        self
    }
}

// MARK: Linear PID Modifiers

impl<M, A, T> DriveDistanceAtHeadingFuture<'_, M, Pid, A, T>
where
    M: Arcade,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
{
    /// Modifies this motion's linear PID gains.
    pub const fn with_linear_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.linear_controller.set_gains(kp, ki, kd);
        self
    }

    /// Modifies this motion's linear proportional gain (`kp`).
    pub const fn with_linear_kp(&mut self, kp: f64) -> &mut Self {
        self.linear_controller.set_kp(kp);
        self
    }

    /// Modifies this motion's linear integral gain (`ki`).
    pub const fn with_linear_ki(&mut self, ki: f64) -> &mut Self {
        self.linear_controller.set_ki(ki);
        self
    }

    /// Modifies this motion's linear derivative gain (`kd`).
    pub const fn with_linear_kd(&mut self, kd: f64) -> &mut Self {
        self.linear_controller.set_kd(kd);
        self
    }

    /// Modifies this motion's linear integration range.
    pub const fn with_linear_integration_range(&mut self, integration_range: f64) -> &mut Self {
        self.linear_controller
            .set_integration_range(Some(integration_range));
        self
    }

    /// Removes this motion's linear integration range.
    pub const fn without_linear_integration_range(&mut self) -> &mut Self {
        self.linear_controller.set_integration_range(None);
        self
    }

    /// Modifies this motion's linear output limit.
    pub const fn with_linear_output_limit(&mut self, limit: f64) -> &mut Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }

    /// Removes this motion's linear output limit.
    pub const fn without_linear_output_limit(&mut self) -> &mut Self {
        self.linear_controller.set_output_limit(None);
        self
    }
}

// MARK: Angular PID Modifiers

impl<M, L, T> DriveDistanceAtHeadingFuture<'_, M, L, AngularPid, T>
where
    M: Arcade,
    L: Feedback<Input = f64, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
{
    /// Modifies this motion's angular PID gains.
    pub const fn with_angular_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.angular_controller.set_gains(kp, ki, kd);
        self
    }

    /// Modifies this motion's angular proportional gain (`kp`).
    pub const fn with_angular_kp(&mut self, kp: f64) -> &mut Self {
        self.angular_controller.set_kp(kp);
        self
    }

    /// Modifies this motion's angular integral gain (`ki`).
    pub const fn with_angular_ki(&mut self, ki: f64) -> &mut Self {
        self.angular_controller.set_ki(ki);
        self
    }

    /// Modifies this motion's angular derivative gain (`kd`).
    pub const fn with_angular_kd(&mut self, kd: f64) -> &mut Self {
        self.angular_controller.set_kd(kd);
        self
    }

    /// Modifies this motion's angular integration range.
    pub const fn with_angular_integration_range(&mut self, integration_range: Angle) -> &mut Self {
        self.angular_controller
            .set_integration_range(Some(integration_range));
        self
    }

    /// Modifies this motion's angular output limit.
    pub const fn with_angular_output_limit(&mut self, limit: f64) -> &mut Self {
        self.angular_controller.set_output_limit(Some(limit));
        self
    }

    /// Removes this motion's angular integration range.
    pub const fn without_angular_integration_range(&mut self) -> &mut Self {
        self.angular_controller.set_integration_range(None);
        self
    }

    /// Removes this motion's angular output limit.
    pub const fn without_angular_output_limit(&mut self) -> &mut Self {
        self.angular_controller.set_output_limit(None);
        self
    }
}
