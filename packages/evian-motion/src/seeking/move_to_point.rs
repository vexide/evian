use core::{
    f64::consts::{FRAC_PI_2, PI},
    future::Future,
    pin::Pin,
    task::Poll,
    time::Duration,
};

use vexide::{
    devices::smart::{Motor, distance},
    io::println,
    time::{Instant, Sleep, sleep},
};

use evian_control::{
    Tolerances,
    loops::{AngularPid, Feedback, Pid},
};
use evian_drivetrain::Drivetrain;
use evian_drivetrain::differential::{Differential, Voltages};
use evian_math::{Angle, IntoAngle, Vec2};
use evian_tracking::{TracksHeading, TracksPosition, TracksVelocity};

pub(crate) struct State {
    sleep: Sleep,
    close: bool,
    prev_facing_point: bool,
    initial_angle_error: Angle,
    prev_time: Instant,
    start_time: Instant,
}

/// Moves the robot to a point using two seeking feedback controllers.
pub struct MoveToPointFuture<'a, L, A, T>
where
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    pub(crate) target_point: Vec2<f64>,
    pub(crate) reverse: bool,
    pub(crate) timeout: Option<Duration>,
    pub(crate) tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
    pub(crate) drivetrain: &'a mut Drivetrain<Differential, T>,
    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<L, A, T> Future for MoveToPointFuture<'_, L, A, T>
where
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();
            let angle_error = (this.drivetrain.tracking.heading()
                - (this.target_point - this.drivetrain.tracking.position())
                    .angle()
                    .rad())
            .wrapped();

            State {
                sleep: sleep(Duration::from_millis(5)),
                start_time: now,
                prev_time: now,
                close: false,
                initial_angle_error: angle_error,
                prev_facing_point: (angle_error.as_radians().abs() < FRAC_PI_2) ^ this.reverse,
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = state.prev_time.elapsed();

        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        let local_target = this.target_point - position;

        let mut distance_error = local_target.length();

        if distance_error.abs() < 7.5 && !state.close {
            state.close = true;
        }
        
        let mut angle_error = (heading - local_target.angle().rad()).wrapped();
        
        if this.reverse {
            distance_error *= -1.0;
            angle_error = (PI.rad() - angle_error).wrapped();
        }
        
        if this
            .tolerances
            .check(distance_error, this.drivetrain.tracking.linear_velocity())
            || this
                .timeout
                .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            _ = this.drivetrain.motors.set_voltages((0.0, 0.0));
            return Poll::Ready(());
        }

        let angular_output = if state.close {
            0.0
        } else {
            this.angular_controller
                .update(-angle_error, Angle::ZERO, dt)
        };
        let linear_output =
            this.linear_controller.update(-distance_error, 0.0, dt) * angle_error.cos();

        _ = this.drivetrain.motors.set_voltages(
            Voltages::from_arcade(linear_output, angular_output).normalized(Motor::V5_MAX_VOLTAGE),
        ); 

        state.sleep = sleep(Duration::from_millis(5));
        state.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Generic Modifiers

impl<L, A, T> MoveToPointFuture<'_, L, A, T>
where
    L: Feedback<Input = f64, Output = f64> + Unpin,
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    /// Reverses this motion, moving to the point backwards rather than forwards.
    pub fn reverse(&mut self) -> &mut Self {
        self.reverse = true;
        self
    }

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
    pub const fn without_tolerance_duration(&mut self) -> &mut Self {
        self.tolerances.duration = None;
        self
    }
}

// MARK: Linear PID Modifiers

impl<A, T> MoveToPointFuture<'_, Pid, A, T>
where
    A: Feedback<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
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

impl<L, T> MoveToPointFuture<'_, L, AngularPid, T>
where
    L: Feedback<Input = f64, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
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
