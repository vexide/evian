use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
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
use evian_drivetrain::{
    Drivetrain,
    differential::{Differential, Voltages},
};
use evian_math::{Angle, IntoAngle, Vec2};
use evian_tracking::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};

/// Feedback-driven driving and turning motion.
#[derive(PartialEq)]
pub struct Basic<
    L: ControlLoop<Input = f64, Output = f64> + Unpin + Clone,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin + Clone,
> {
    /// Linear (forward driving) feedback controller.
    pub linear_controller: L,

    /// Angular (turning) feedback controller.
    pub angular_controller: A,

    /// Linear settling conditions.
    pub linear_tolerances: Tolerances,

    /// Angular settling conditions.
    pub angular_tolerances: Tolerances,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin + Clone,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin + Clone,
> Basic<L, A>
{
    /// Moves the robot forwards by a given distance (measured in wheel units) while
    /// turning to face a heading.
    ///
    /// Negative `target_distance` values will move the robot backwards.
    pub fn drive_distance_at_heading<
        'a,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        target_distance: f64,
        target_heading: Angle,
    ) -> DriveDistanceAtHeadingFuture<'a, L, A, T> {
        DriveDistanceAtHeadingFuture {
            sleep: sleep(Duration::from_millis(5)),
            target_distance,
            target_heading,
            start_time: Instant::now(),
            prev_time: Instant::now(),
            timeout: self.timeout,
            initial_forward_travel: drivetrain.tracking.forward_travel(),
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_settled: false,
            angular_settled: false,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
        }
    }

    /// Moves the robot forwards by a given distance (measured in wheel units).
    ///
    /// Negative `distance` values will move the robot backwards.
    pub fn drive_distance<'a, T: TracksForwardTravel + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        distance: f64,
    ) -> DriveDistanceAtHeadingFuture<'a, L, A, T> {
        self.drive_distance_at_heading(drivetrain, distance, drivetrain.tracking.heading())
    }

    /// Turns the robot in place to face a heading.
    pub fn turn_to_heading<'a, T: TracksForwardTravel + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        heading: Angle,
    ) -> DriveDistanceAtHeadingFuture<'a, L, A, T> {
        self.drive_distance_at_heading(drivetrain, 0.0, heading)
    }

    /// Turns the robot in place to face a 2D point.
    pub fn turn_to_point<
        'a,
        T: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        point: impl Into<Vec2<f64>>,
    ) -> TurnToPointFuture<'a, L, A, T> {
        TurnToPointFuture {
            sleep: sleep(Duration::from_millis(5)),
            point: point.into(),
            start_time: Instant::now(),
            prev_time: Instant::now(),
            timeout: self.timeout,
            initial_forward_travel: drivetrain.tracking.forward_travel(),
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_settled: false,
            angular_settled: false,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
        }
    }
}

pub struct DriveDistanceAtHeadingFuture<
    'a,
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
> {
    sleep: Sleep,
    target_distance: f64,
    target_heading: Angle,
    start_time: Instant,
    prev_time: Instant,
    timeout: Option<Duration>,
    initial_forward_travel: f64,
    linear_tolerances: Tolerances,
    angular_tolerances: Tolerances,
    linear_controller: L,
    angular_controller: A,
    linear_settled: bool,
    angular_settled: bool,
    drivetrain: &'a mut Drivetrain<Differential, T>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
> Future for DriveDistanceAtHeadingFuture<'_, L, A, T>
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        if Pin::new(&mut this.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = this.prev_time.elapsed();

        let forward_travel = this.drivetrain.tracking.forward_travel();
        let heading = this.drivetrain.tracking.heading();

        let linear_error = (this.target_distance + this.initial_forward_travel) - forward_travel;
        let angular_error = (this.target_heading - heading).wrapped();

        if this
            .linear_tolerances
            .check(linear_error, this.drivetrain.tracking.linear_velocity())
        {
            this.linear_settled = true;
        }
        if this.angular_tolerances.check(
            angular_error.as_radians(),
            this.drivetrain.tracking.angular_velocity(),
        ) {
            this.angular_settled = true;
        }

        if (this.linear_settled && this.angular_settled)
            || this
                .timeout
                .is_some_and(|timeout| this.start_time.elapsed() > timeout)
        {
            _ = this.drivetrain.motors.set_voltages((0.0, 0.0));
            return Poll::Ready(());
        }

        let linear_output = this.linear_controller.update(
            forward_travel,
            this.target_distance + this.initial_forward_travel,
            dt,
        );
        let angular_output = this
            .angular_controller
            .update(heading, this.target_heading, dt);

        _ = this.drivetrain.motors.set_voltages(
            Voltages(
                linear_output + angular_output,
                linear_output - angular_output,
            )
            .normalized(Motor::V5_MAX_VOLTAGE),
        );

        this.sleep = sleep(Duration::from_millis(5));
        this.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
> DriveDistanceAtHeadingFuture<'_, L, A, T>
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

    pub const fn with_linear_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.linear_tolerances = tolerances;
        self
    }

    pub const fn with_linear_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_linear_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_linear_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.linear_tolerances.duration = Some(duration);
        self
    }

    pub const fn with_angular_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.angular_tolerances = tolerances;
        self
    }

    pub const fn with_angular_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_angular_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_angular_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.angular_tolerances.duration = Some(duration);
        self
    }
}

impl<
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
> DriveDistanceAtHeadingFuture<'_, Pid, A, T>
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
    T: TracksForwardTravel + TracksHeading + TracksVelocity,
> DriveDistanceAtHeadingFuture<'_, L, AngularPid, T>
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

pub struct TurnToPointFuture<
    'a,
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
> {
    sleep: Sleep,
    point: Vec2<f64>,
    start_time: Instant,
    prev_time: Instant,
    timeout: Option<Duration>,
    initial_forward_travel: f64,
    linear_tolerances: Tolerances,
    angular_tolerances: Tolerances,
    linear_controller: L,
    angular_controller: A,
    linear_settled: bool,
    angular_settled: bool,
    drivetrain: &'a mut Drivetrain<Differential, T>,
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity + TracksPosition,
> Future for TurnToPointFuture<'_, L, A, T>
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        if Pin::new(&mut this.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = this.prev_time.elapsed();

        let forward_travel = this.drivetrain.tracking.forward_travel();
        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();
        let target_heading = (this.point - position).angle().rad();

        let linear_error = this.initial_forward_travel - forward_travel;
        let angular_error = (heading - target_heading).wrapped();

        if this
            .linear_tolerances
            .check(linear_error, this.drivetrain.tracking.linear_velocity())
        {
            this.linear_settled = true;
        }
        if this.angular_tolerances.check(
            angular_error.as_radians(),
            this.drivetrain.tracking.angular_velocity(),
        ) {
            this.angular_settled = true;
        }

        if (this.linear_settled && this.angular_settled)
            || this
                .timeout
                .is_some_and(|timeout| this.start_time.elapsed() > timeout)
        {
            _ = this.drivetrain.motors.set_voltages((0.0, 0.0));
            return Poll::Ready(());
        }

        let linear_output =
            this.linear_controller
                .update(forward_travel, this.initial_forward_travel, dt);
        let angular_output = this
            .angular_controller
            .update(-angular_error, Angle::ZERO, dt);

        _ = this.drivetrain.motors.set_voltages(
            Voltages(
                linear_output + angular_output,
                linear_output - angular_output,
            )
            .normalized(Motor::V5_MAX_VOLTAGE),
        );

        this.sleep = sleep(Duration::from_millis(5));
        this.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

impl<
    L: ControlLoop<Input = f64, Output = f64> + Unpin,
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity + TracksPosition,
> TurnToPointFuture<'_, L, A, T>
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

    pub const fn with_linear_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.linear_tolerances = tolerances;
        self
    }

    pub const fn with_linear_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_linear_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_linear_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.linear_tolerances.duration = Some(duration);
        self
    }

    pub const fn with_angular_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.angular_tolerances = tolerances;
        self
    }

    pub const fn with_angular_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.error_tolerance = Some(tolerance);
        self
    }

    pub const fn with_angular_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    pub const fn with_angular_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.angular_tolerances.duration = Some(duration);
        self
    }
}

impl<
    A: ControlLoop<Input = Angle, Output = f64> + Unpin,
    T: TracksForwardTravel + TracksHeading + TracksVelocity + TracksPosition,
> TurnToPointFuture<'_, Pid, A, T>
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
    T: TracksPosition + TracksForwardTravel + TracksHeading + TracksVelocity,
> TurnToPointFuture<'_, L, AngularPid, T>
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
