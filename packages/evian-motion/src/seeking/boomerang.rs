use core::{
    f64::consts::{FRAC_PI_2, PI},
    future::Future,
    pin::Pin,
    task::Poll,
    time::Duration,
};

use vexide::{
    devices::smart::Motor,
    time::{Instant, Sleep, sleep},
};

use evian_control::{
    Tolerances,
    loops::{AngularPid, ControlLoop, Pid},
};
use evian_drivetrain::Drivetrain;
use evian_drivetrain::differential::{Differential, Voltages};
use evian_math::{Angle, IntoAngle, Vec2};
use evian_tracking::{TracksHeading, TracksPosition, TracksVelocity};

pub struct State {
    sleep: Sleep,
    prev_time: Instant,
    start_time: Instant,
    prev_position: Vec2<f64>,
}

/// Boomerang move-to-pose algorithm.
pub struct BoomerangFuture<
    'a,
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> {
    pub(crate) target_point: Vec2<f64>,
    pub(crate) target_heading: Angle,
    pub(crate) lead: f64,
    pub(crate) timeout: Option<Duration>,
    pub(crate) tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
    pub(crate) drivetrain: &'a mut Drivetrain<Differential, T>,

    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> Future for BoomerangFuture<'_, L, A, T>
{
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();
            State {
                sleep: sleep(Duration::from_millis(5)),
                prev_position: this.drivetrain.tracking.position(),
                start_time: now,
                prev_time: now,
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = state.prev_time.elapsed();

        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        let carrot = this.target_point
            - Vec2::from_polar(
                position.distance(this.target_point) * this.lead,
                this.target_heading.as_radians(),
            );

        let local_target = carrot - position;

        let mut angular_error = (heading - local_target.angle().rad()).wrapped();
        let mut linear_error = local_target.length();

        if angular_error.as_radians().abs() > FRAC_PI_2 {
            linear_error *= -1.0;
            angular_error = (PI.rad() - angular_error).wrapped();
        }

        if this
            .tolerances
            .check(linear_error, this.drivetrain.tracking.linear_velocity())
            || this
                .timeout
                .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            _ = this.drivetrain.motors.set_voltages((0.0, 0.0));
            return Poll::Ready(());
        }

        let angular_output = this
            .angular_controller
            .update(-angular_error, Angle::ZERO, dt);
        let linear_output =
            this.linear_controller.update(-linear_error, 0.0, dt) * angular_error.cos();

        _ = this.drivetrain.motors.set_voltages(
            Voltages::from_arcade(linear_output, angular_output).normalized(Motor::V5_MAX_VOLTAGE),
        );

        state.sleep = sleep(Duration::from_millis(5));
        state.prev_time = Instant::now();
        state.prev_position = position;

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Generic Modifiers

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, L, A, T>
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

    /// Modifies this motion's tolerances.
    pub const fn with_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.tolerances = tolerances;
        self
    }

    /// Modifies this motion's error tolerance.
    pub const fn with_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's error tolerance.
    pub const fn withear_error_tolerance(&mut self) -> &mut Self {
        self.tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's velocity tolerance.
    pub const fn with_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's velocity tolerance.
    pub const fn withear_velocity_tolerance(&mut self) -> &mut Self {
        self.tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's tolerance duration.
    pub const fn with_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's tolerance duration.
    pub const fn withear_tolerance_duration(&mut self) -> &mut Self {
        self.tolerances.duration = None;
        self
    }
}

// MARK: Linear PID Modifiers

impl<
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, Pid, A, T>
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

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, L, AngularPid, T>
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