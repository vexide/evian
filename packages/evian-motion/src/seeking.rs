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

/// Point-to-Point Feedback Seeking
///
/// This struct provides implementations of adaptive feedback seeking algorithms, which
/// utilize two feedback controllers (one for straight driving and one for turning) to
/// reach a desired point. This is most commonly done using two PID controllers.
///
/// Seeking motions include:
/// - [`move_to_point`](Seeking::move_to_point), which moves the drivetrain to a desired point.
/// - [`boomerang`](Seeking::move_to_point), which moves the drivetrain to a desired pose (including heading).
#[derive(PartialEq)]
pub struct Seeking<
    L: ControlLoop<Input = f64, Output = f64> + Unpin + Clone,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin + Clone,
> {
    /// Linear (forward driving) feedback controller.
    pub linear_controller: L,

    /// Angular (turning) feedback controller.
    pub angular_controller: A,

    /// Linear settling conditions.
    ///
    /// Error is denoted by the distance from the target, while velocity
    /// is the robot's linear forward velocity.
    pub tolerances: Tolerances,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin + Clone,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin + Clone,
> Seeking<L, A>
{
    /// Moves the robot to a 2D point.
    ///
    /// The final heading of the robot after this motion executes is undefined.
    /// For full pose control, use [`Seeking::boomerang`].
    pub fn move_to_point<'a, T: TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
    ) -> MoveToPointFuture<'a, L, A, T> {
        MoveToPointFuture {
            sleep: sleep(Duration::from_millis(5)),
            drivetrain,
            target_point: point.into(),
            start_time: Instant::now(),
            prev_time: Instant::now(),
            timeout: self.timeout,
            tolerances: self.tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
        }
    }

    /// Moves the robot to a desired pose (position and heading).
    ///
    /// This motion uses a boomerang controller, which is a motion algorithm
    /// for moving differential drivetrains to a desired pose. Larger `lead`
    /// values will result in wider arcs, while smaller `lead` values will
    /// result in smaller arcs. You may need to tune the `lead` value in order
    /// to properly reach the desired heading by the end of the motion.
    pub fn boomerang<'a, T: TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
        heading: Angle,
        lead: f64,
    ) -> BoomerangFuture<'a, L, A, T> {
        BoomerangFuture {
            sleep: sleep(Duration::from_millis(5)),
            prev_position: drivetrain.tracking.position(),
            drivetrain,
            target_heading: heading,
            lead,
            target_point: point.into(),
            start_time: Instant::now(),
            prev_time: Instant::now(),
            timeout: self.timeout,
            tolerances: self.tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
        }
    }
}

pub struct MoveToPointFuture<
    'a,
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> {
    sleep: Sleep,
    target_point: Vec2<f64>,
    start_time: Instant,
    prev_time: Instant,
    timeout: Option<Duration>,
    tolerances: Tolerances,
    linear_controller: L,
    angular_controller: A,
    drivetrain: &'a mut Drivetrain<Differential, T>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> MoveToPointFuture<'_, L, A, T>
{
    pub fn with_linear_controller(&mut self, controller: L) -> &mut Self {
        self.linear_controller = controller;
        self
    }

    pub fn with_angular_controller(&mut self, controller: A) -> &mut Self {
        self.angular_controller = controller;
        self
    }

    pub const fn with_timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = Some(timeout);
        self
    }

    pub const fn with_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.tolerances = tolerances;
        self
    }

    pub const fn with_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.tolerances.duration = Some(duration);
        self
    }
}

impl<
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> MoveToPointFuture<'_, Pid, A, T>
{
    pub const fn with_linear_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.linear_controller.set_gains(kp, ki, kd);
        self
    }

    pub const fn with_linear_kp(&mut self, kp: f64) -> &mut Self {
        self.linear_controller.set_kp(kp);
        self
    }

    pub const fn with_linear_ki(&mut self, ki: f64) -> &mut Self {
        self.linear_controller.set_ki(ki);
        self
    }

    pub const fn with_linear_kd(&mut self, kd: f64) -> &mut Self {
        self.linear_controller.set_kd(kd);
        self
    }

    pub const fn with_linear_integration_range(&mut self, integration_range: f64) -> &mut Self {
        self.linear_controller
            .set_integration_range(Some(integration_range));
        self
    }

    pub const fn with_linear_output_limit(&mut self, limit: f64) -> &mut Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> MoveToPointFuture<'_, L, AngularPid, T>
{
    pub const fn with_angular_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.angular_controller.set_gains(kp, ki, kd);
        self
    }

    pub const fn with_angular_kp(&mut self, kp: f64) -> &mut Self {
        self.angular_controller.set_kp(kp);
        self
    }

    pub const fn with_angular_ki(&mut self, ki: f64) -> &mut Self {
        self.angular_controller.set_ki(ki);
        self
    }

    pub const fn with_angular_kd(&mut self, kd: f64) -> &mut Self {
        self.angular_controller.set_kd(kd);
        self
    }

    pub const fn with_angular_integration_range(&mut self, integration_range: Angle) -> &mut Self {
        self.angular_controller
            .set_integration_range(Some(integration_range));
        self
    }

    pub const fn with_angular_output_limit(&mut self, limit: f64) -> &mut Self {
        self.angular_controller.set_output_limit(Some(limit));
        self
    }
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> Future for MoveToPointFuture<'_, L, A, T>
{
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = self.get_mut();

        if Pin::new(&mut this.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = this.prev_time.elapsed();

        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        let local_target = this.target_point - position;

        let mut distance_error = local_target.length();
        let mut angle_error = (heading - local_target.angle().rad()).wrapped();

        if angle_error.as_radians().abs() > FRAC_PI_2 {
            distance_error *= -1.0;
            angle_error = (PI.rad() - angle_error).wrapped();
        }

        if this
            .tolerances
            .check(distance_error, this.drivetrain.tracking.linear_velocity())
            || this
                .timeout
                .is_some_and(|timeout| this.start_time.elapsed() > timeout)
        {
            _ = this.drivetrain.motors.set_voltages((0.0, 0.0));
            return Poll::Ready(());
        }

        let angular_output = this
            .angular_controller
            .update(-angle_error, Angle::ZERO, dt);
        let linear_output =
            this.linear_controller.update(-distance_error, 0.0, dt) * angle_error.cos();

        _ = this.drivetrain.motors.set_voltages(
            Voltages::from_arcade(linear_output, angular_output).normalized(Motor::V5_MAX_VOLTAGE),
        );

        this.sleep = sleep(Duration::from_millis(5));
        this.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

pub struct BoomerangFuture<
    'a,
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> {
    sleep: Sleep,
    target_point: Vec2<f64>,
    target_heading: Angle,
    start_time: Instant,
    prev_time: Instant,
    prev_position: Vec2<f64>,
    lead: f64,
    timeout: Option<Duration>,
    tolerances: Tolerances,
    linear_controller: L,
    angular_controller: A,
    drivetrain: &'a mut Drivetrain<Differential, T>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, L, A, T>
{
    pub fn with_linear_controller(&mut self, controller: L) -> &mut Self {
        self.linear_controller = controller;
        self
    }

    pub fn with_angular_controller(&mut self, controller: A) -> &mut Self {
        self.angular_controller = controller;
        self
    }

    pub const fn with_timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = Some(timeout);
        self
    }

    pub const fn with_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.tolerances = tolerances;
        self
    }

    pub const fn with_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.tolerances.duration = Some(duration);
        self
    }
}

impl<
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, Pid, A, T>
{
    pub const fn with_linear_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.linear_controller.set_gains(kp, ki, kd);
        self
    }

    pub const fn with_linear_kp(&mut self, kp: f64) -> &mut Self {
        self.linear_controller.set_kp(kp);
        self
    }

    pub const fn with_linear_ki(&mut self, ki: f64) -> &mut Self {
        self.linear_controller.set_ki(ki);
        self
    }

    pub const fn with_linear_kd(&mut self, kd: f64) -> &mut Self {
        self.linear_controller.set_kd(kd);
        self
    }

    pub const fn with_linear_integration_range(&mut self, integration_range: f64) -> &mut Self {
        self.linear_controller
            .set_integration_range(Some(integration_range));
        self
    }

    pub const fn with_linear_output_limit(&mut self, limit: f64) -> &mut Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> BoomerangFuture<'_, L, AngularPid, T>
{
    pub const fn with_angular_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.angular_controller.set_gains(kp, ki, kd);
        self
    }

    pub const fn with_angular_kp(&mut self, kp: f64) -> &mut Self {
        self.angular_controller.set_kp(kp);
        self
    }

    pub const fn with_angular_ki(&mut self, ki: f64) -> &mut Self {
        self.angular_controller.set_ki(ki);
        self
    }

    pub const fn with_angular_kd(&mut self, kd: f64) -> &mut Self {
        self.angular_controller.set_kd(kd);
        self
    }

    pub const fn with_angular_integration_range(&mut self, integration_range: Angle) -> &mut Self {
        self.angular_controller
            .set_integration_range(Some(integration_range));
        self
    }

    pub const fn with_angular_output_limit(&mut self, limit: f64) -> &mut Self {
        self.angular_controller.set_output_limit(Some(limit));
        self
    }
}

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

        if Pin::new(&mut this.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = this.prev_time.elapsed();

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
                .is_some_and(|timeout| this.start_time.elapsed() > timeout)
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

        this.sleep = sleep(Duration::from_millis(5));
        this.prev_time = Instant::now();
        this.prev_position = position;

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}
