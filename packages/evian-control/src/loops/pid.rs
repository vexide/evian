use std::time::Duration;

use evian_math::Angle;

use super::Feedback;

/// The kind of quantity a [`Pid`] controller regulates.
///
/// Sign-based integral reset applies only in [`Position`](PidMode::Position) mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PidMode {
    /// Regulates a position. Sign-based integral reset is enabled.
    Position,
    /// Regulates a velocity. Sign-based integral reset is disabled.
    Velocity,
}

// MARK: Linear Controller

/// PID controller.
///
/// The PID controller is one of the most widely used feedback control algorithms in industrial
/// control systems, robotics, and process control. It computes a control signal based on three
/// terms: **proportional**, **integral**, and **derivative**.
///
/// # Components
///
/// - The **proportional** term produces an output proportional to the current error, multiplied by
///   the gain constant `kp`. This provides the main control action: larger errors result in larger
///   corrective responses and vice versa.
///
/// - The **integral** term sums the error over time, multiplied by the gain constant `ki`. This
///   term eliminates *steady-state error* by ensuring that even small errors will eventually
///   accumulate a large enough response to reach the setpoint. Without integral action, a system
///   might stabilize with a small, persistent error if the control signal becomes too small to
///   overcome system friction, gravity, or other external factors on the system. For example, a
///   motor might need some minimum voltage to start moving, or a drone might need extra thrust to
///   hover against gravity. The integral term accumulates over time to provide this additional
///   correction.
///
/// - The **derivative** term measures the error's change over time, multiplied by the gain constant
///   `kd`. This provides a damping effect that reduces overshoot and oscillation by counteracting
///   rapid changes in error. The derivative term helps anticipate and smooth out the system's
///   response, preventing sudden changes resulting from large proportional or integral gains.
///
/// # Tuning
///
/// Tuning a PID controller requires adjusting `kp`, `ki`, and `kd` to allow the system to reach a
/// setpoint in a reasonable amount of time without oscillations (rapid, unpredictable changes in
/// output).
///
/// Tuning methods are typically dependent on the application that the PID controller is used in,
/// but a common method is as follows:
///
/// 1. Start with all gains at zero (`kp = 0.0`, `ki = 0.0`, `kd = 0.0`).
///
/// 2. Tune proportional gain first:
///    - Gradually increase `kp` until the system starts to oscillate around the setpoint.
///    - *Oscillation* occurs when the system reaches and overshoots the setpoint, then repeatedly
///      overadjusts itself around the setpoint, resulting in a "back-and-forth" motion around the
///      setpoint.
///
/// 3. Tune the derivative gain:
///    - Start with a very small `kd` gain (0.05 × `kp` or less is a safe bet to start with).
///    - Gradually increase by small increments until oscillations from the proportional term stop
///      occurring.
///
/// 4. Add integral gain if necessary:
///    - Integral gain is only necessary if your controller's proportional and derivative terms
///      become small enough to where they can no longer overcome some external factor (such as
///      friction) of the system, resulting in what's called *steady-state error*.
///    - Start with a very small `ki` gain (such as 0.01 × `kp`).
///    - Increase `ki` slowly until steady-state errors are eliminated within an acceptable time.
///    - If oscillation occurs, reduce both `ki` and `kp` slightly.
///
/// Common signs of poor tuning:
///
/// - Slow response: `kp` is too low.
/// - Excessive overshoot: `kd` is too low or `ki` is too high.
/// - Oscillation: `kp` is too high or `kd` is too low.
/// - Noisy, unpredictable response: `kd` is too high.
///
/// # Integral Windup (and Mitigations)
///
/// In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
/// reaches a saturation point preventing the error from decreasing. In this case, integral will
/// rapidly accumulate, causing large and unpredictable control signals. This specific
/// implementation provides four mitigations for integral windup:
///
/// 1. **Sign-based reset:** When the sign of error changes (in other words, when the controller has
///    crossed/overshot its target), the integral value is reset to prevent overshoot of the target.
///    Applies only in [`Position`](PidMode::Position) mode.
/// 2. **Integration bounds:** An optional `integration_range` value can be passed to the
///    controller, which defines a range of error where integration will occur. When
///    `|error| > integration_range`, no integration will occur if used.
/// 3. **Integral clamp:** The accumulated integral is clamped to `±max_integral` in any mode.
/// 4. **Conditional integration:** While the output is saturated at `output_limit`, the integral
///    holds its previous value instead of accumulating when doing so would push the output further
///    past the limit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,

    mode: PidMode,
    integral: f64,
    integration_range: Option<f64>,
    max_integral: f64,
    output_limit: Option<f64>,
    prev_error: f64,
    prev_measurement: f64,
    initialized: bool,
}

impl Pid {
    /// Construct a new PID controller from gain constants and an optional integration range.
    ///
    /// The integral limit defaults to [`f64::INFINITY`] (unbounded); set it with
    /// [`set_max_integral`](Self::set_max_integral).
    #[must_use]
    pub const fn new(kp: f64, ki: f64, kd: f64, integration_range: Option<f64>) -> Self {
        Self {
            kp,
            ki,
            kd,
            mode: PidMode::Position,
            integration_range,
            max_integral: f64::INFINITY,
            output_limit: None,
            integral: 0.0,
            prev_error: 0.0,
            prev_measurement: 0.0,
            initialized: false,
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    #[must_use]
    pub const fn gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Returns the controller's proportional gain (`kp`).
    #[must_use]
    pub const fn kp(&self) -> f64 {
        self.kp
    }

    /// Returns the controller's integral gain (`ki`).
    #[must_use]
    pub const fn ki(&self) -> f64 {
        self.ki
    }

    /// Returns the controller's derivative gain (`kd`).
    #[must_use]
    pub const fn kd(&self) -> f64 {
        self.kd
    }

    /// Returns the controller's mode.
    #[must_use]
    pub const fn mode(&self) -> PidMode {
        self.mode
    }

    /// Returns the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    #[must_use]
    pub const fn integration_range(&self) -> Option<f64> {
        self.integration_range
    }

    /// Returns the controller's integral limit.
    #[must_use]
    pub const fn max_integral(&self) -> f64 {
        self.max_integral
    }

    /// Returns the controller's output limit, or `None` if there is no
    /// limit applied.
    #[must_use]
    pub const fn output_limit(&self) -> Option<f64> {
        self.output_limit
    }

    /// Sets the PID gains to provided values.
    pub const fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Sets the controller's proportional gain (`kp`).
    pub const fn set_kp(&mut self, kp: f64) {
        self.kp = kp;
    }

    /// Sets the controller's integral gain (`ki`).
    pub const fn set_ki(&mut self, ki: f64) {
        self.ki = ki;
    }

    /// Sets the controller's derivative gain (`kd`).
    pub const fn set_kd(&mut self, kd: f64) {
        self.kd = kd;
    }

    /// Sets the controller's mode.
    pub const fn set_mode(&mut self, mode: PidMode) {
        self.mode = mode;
    }

    /// Sets the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    pub const fn set_integration_range(&mut self, range: Option<f64>) {
        self.integration_range = range;
    }

    /// Sets the controller's integral limit. Use [`f64::INFINITY`] for no limit.
    pub const fn set_max_integral(&mut self, limit: f64) {
        self.max_integral = limit;
    }

    /// Sets the controller's output limit.
    ///
    /// This sets a maximum range for the controller's output signal. It will effectively limit how
    /// fast the controller is able to drive the system, which may be desirable in some cases (e.g.
    /// limiting the maximum speed of a robot's motion).
    pub const fn set_output_limit(&mut self, range: Option<f64>) {
        self.output_limit = range;
    }

    /// Resets controller state between movements.
    ///
    /// Clears the integral and marks the controller uninitialized, so the next
    /// [`update`](Feedback::update) reseeds its derivative/error history instead of producing a
    /// spurious first-tick derivative.
    pub const fn reset(&mut self) {
        self.integral = 0.0;
        self.initialized = false;
    }
}

// MARK: Loop

impl Feedback for Pid {
    type State = f64;
    type Signal = f64;

    fn update(&mut self, measurement: f64, setpoint: f64, dt: Duration) -> f64 {
        let error = setpoint - measurement;

        // The first update after construction or a reset has no previous sample. Seed the history
        // with the current values so this tick's derivative is 0 and the overshoot reset is a no-op.
        if !self.initialized {
            self.initialized = true;
            self.prev_error = error;
            self.prev_measurement = measurement;
        }

        // 1. Band gate. Accumulate only inside the integration range.
        let mut integral_candidate = if self
            .integration_range
            .is_none_or(|range| error.abs() < range)
        {
            self.integral + error * dt.as_secs_f64()
        } else {
            0.0
        };

        // 2. Overshoot reset, position mode only. A velocity controller's error sign flips
        //    constantly while tracking, which would wreck the integral.
        if self.mode == PidMode::Position && error.signum() != self.prev_error.signum() {
            integral_candidate = 0.0;
        }

        // 3. Windup clamp. Bound the integral magnitude in any mode; a no-op when the limit
        //    is infinite.
        integral_candidate = integral_candidate.clamp(-self.max_integral, self.max_integral);

        // Derivative of the measurement, not the error, to avoid derivative kick on setpoint changes.
        let derivative = (self.prev_measurement - measurement) / dt.as_secs_f64();
        self.prev_error = error;
        self.prev_measurement = measurement;

        // Control signal = error * kp + integral * ki + derivative * kd.
        let mut output =
            (error * self.kp) + (integral_candidate * self.ki) + (derivative * self.kd);

        // 4. Conditional integration. Commit the new integral unless the output is saturated and
        //    the integral would push it further past the limit.
        if self.output_limit.is_some_and(|limit| output.abs() >= limit)
            && output.signum() == error.signum()
        {
            output = (error * self.kp) + (self.integral * self.ki) + (derivative * self.kd);
        } else {
            self.integral = integral_candidate;
        }

        if let Some(range) = self.output_limit {
            output = output.clamp(-range, range);
        }

        output
    }
}

// MARK: Angular Controller

/// PID controller for use in rotational systems.
///
/// This struct operates on the same principles and implementation as [`Pid`], but takes exclusively
/// [`Angle`]s as input. Unlike [`Pid`], [`AngularPid`] is able to recognize when angles *wrap*.
/// This means a 0° measurement is equivalent to a 360° measurement, for instance.
///
/// This is useful for cases where you want the controller to drive the system to its setpoint using
/// the "shortest turn possible".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularPid {
    kp: f64,
    ki: f64,
    kd: f64,

    mode: PidMode,
    integral: f64,
    integration_range: Option<Angle>,
    max_integral: f64,
    output_limit: Option<f64>,
    prev_error: Angle,
    prev_measurement: Angle,
    initialized: bool,
}

impl AngularPid {
    /// Construct a new PID controller from gain constants and an optional integration range.
    ///
    /// The integral limit defaults to [`f64::INFINITY`] (unbounded); set it with
    /// [`set_max_integral`](Self::set_max_integral).
    #[must_use]
    pub const fn new(kp: f64, ki: f64, kd: f64, integration_range: Option<Angle>) -> Self {
        Self {
            kp,
            ki,
            kd,
            mode: PidMode::Position,
            integration_range,
            max_integral: f64::INFINITY,
            integral: 0.0,
            output_limit: None,
            prev_error: Angle::from_radians(0.0),
            prev_measurement: Angle::from_radians(0.0),
            initialized: false,
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    #[must_use]
    pub const fn gains(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Returns the controller's proportional gain (`kp`).
    #[must_use]
    pub const fn kp(&self) -> f64 {
        self.kp
    }

    /// Returns the controller's integral gain (`ki`).
    #[must_use]
    pub const fn ki(&self) -> f64 {
        self.ki
    }

    /// Returns the controller's derivative gain (`kd`).
    #[must_use]
    pub const fn kd(&self) -> f64 {
        self.kd
    }

    /// Returns the controller's mode.
    #[must_use]
    pub const fn mode(&self) -> PidMode {
        self.mode
    }

    /// Returns the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    #[must_use]
    pub const fn integration_range(&self) -> Option<Angle> {
        self.integration_range
    }

    /// Returns the controller's integral limit.
    #[must_use]
    pub const fn max_integral(&self) -> f64 {
        self.max_integral
    }

    /// Returns the controller's output limit, or `None` if there is no limit applied.
    #[must_use]
    pub const fn output_limit(&self) -> Option<f64> {
        self.output_limit
    }

    /// Sets the PID gains to provided values.
    pub const fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Sets the controller's proportional gain (`kp`).
    pub const fn set_kp(&mut self, kp: f64) {
        self.kp = kp;
    }

    /// Sets the controller's integral gain (`ki`).
    pub const fn set_ki(&mut self, ki: f64) {
        self.ki = ki;
    }

    /// Sets the controller's derivative gain (`kd`).
    pub const fn set_kd(&mut self, kd: f64) {
        self.kd = kd;
    }

    /// Sets the controller's mode.
    pub const fn set_mode(&mut self, mode: PidMode) {
        self.mode = mode;
    }

    /// Sets the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    pub const fn set_integration_range(&mut self, range: Option<Angle>) {
        self.integration_range = range;
    }

    /// Sets the controller's integral limit. Use [`f64::INFINITY`] for no limit.
    pub const fn set_max_integral(&mut self, limit: f64) {
        self.max_integral = limit;
    }

    /// Sets the controller's output limit.
    ///
    /// This sets a maximum range for the controller's output signal. It will effectively limit how
    /// fast the controller is able to drive the system, which may be desirable in some cases (e.g.
    /// limiting the maximum speed of a robot's motion).
    pub const fn set_output_limit(&mut self, range: Option<f64>) {
        self.output_limit = range;
    }

    /// Resets controller state between movements.
    ///
    /// Clears the integral and marks the controller uninitialized, so the next
    /// [`update`](Feedback::update) reseeds its derivative/error history instead of producing a
    /// spurious first-tick derivative.
    pub const fn reset(&mut self) {
        self.integral = 0.0;
        self.initialized = false;
    }
}

// MARK: Loop

impl Feedback for AngularPid {
    type State = Angle;
    type Signal = f64;

    fn update(&mut self, measurement: Angle, setpoint: Angle, dt: Duration) -> f64 {
        let error = (setpoint - measurement).wrapped_half();

        // The first update after construction or a reset has no previous sample. Seed the history
        // with the current values so this tick's derivative is 0 and the overshoot reset is a no-op.
        if !self.initialized {
            self.initialized = true;
            self.prev_error = error;
            self.prev_measurement = measurement;
        }

        // 1. Band gate. Accumulate only inside the integration range.
        let mut integral_candidate = if self
            .integration_range
            .is_none_or(|range| error.as_radians().abs() < range.as_radians())
        {
            self.integral + error.as_radians() * dt.as_secs_f64()
        } else {
            0.0
        };

        // 2. Overshoot reset, position mode only. A velocity controller's error sign flips
        //    constantly while tracking, which would wreck the integral.
        #[allow(clippy::float_cmp)]
        if self.mode == PidMode::Position && error.signum() != self.prev_error.signum() {
            integral_candidate = 0.0;
        }

        // 3. Windup clamp. Bound the integral magnitude in any mode; a no-op when the limit
        //    is infinite.
        integral_candidate = integral_candidate.clamp(-self.max_integral, self.max_integral);

        // Derivative of the measurement, not the error, to avoid derivative kick on setpoint changes.
        let derivative = (self.prev_measurement - measurement)
            .wrapped_half()
            .as_radians()
            / dt.as_secs_f64();
        self.prev_error = error;
        self.prev_measurement = measurement;

        // Control signal = error * kp + integral * ki + derivative * kd.
        let mut output = (error.as_radians() * self.kp)
            + (integral_candidate * self.ki)
            + (derivative * self.kd);

        // 4. Conditional integration. Commit the new integral unless the output is saturated and
        //    the integral would push it further past the limit.
        if self.output_limit.is_some_and(|limit| output.abs() >= limit)
            && output.signum() == error.signum()
        {
            output =
                (error.as_radians() * self.kp) + (self.integral * self.ki) + (derivative * self.kd);
        } else {
            self.integral = integral_candidate;
        }

        if let Some(range) = self.output_limit {
            output = output.clamp(-range, range);
        }

        output
    }
}
