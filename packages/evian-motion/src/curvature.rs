use core::f64::consts::FRAC_PI_2;
use vexide::{devices::smart::motor::MotorError, float::Float};

use evian_drivetrain::{
    Drivetrain,
    differential::{Differential, Voltages},
};

/// Curvature Drive (aka Cheesy Drive) Controller
///
/// Curvature Drive is a nonlinear and curvature-based drivetrain control algorithm. Optimized for
/// driver intuition and precise handling, it smooths inputs and adapts to the situation. Unlike
/// other algorithms such as Arcade Drive and Tank Drive, it performs some mathematical computations
/// and accepts some constants and maintains an internal state that changes every time the algorithm
/// is ran using [`CurvatureDrive::update`].
///
/// This implemenation is based on <https://wiki.purduesigbots.com/software/robotics-basics/curvature-cheesy-drive>.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurvatureDrive {
    /// Determines how fast the robot's turn traverses a sine curve, and affects
    /// its nonlinearity.
    pub turn_nonlinearity: f64,

    /// Minimum value for `turn` and `throttle` to not ignore and round down to
    /// zero, creates a deadzone at the center of the joystick
    pub deadzone: f64,

    /// Tunes throttle
    pub slew: f64,

    /// Used to counteract robot inertia while turning to prevent overshooting.
    pub negative_inertia_scalar: f64,

    /// Affects sensitivity of turning power, can be used to slow down or speed
    /// up turning.
    pub turn_sensitivity: f64,

    prev_turn: f64,
    prev_throttle: f64,
    negative_inertia_accumulator: f64,
    quick_stop_accumulator: f64,
}

// On each iteration of the drive loop where we aren't point turning, the accumulators are
// constrained to [-1, 1]
fn update_accumulator(accumulator: &mut f64) {
    if *accumulator > 1.0 {
        *accumulator -= 1.0;
    } else if *accumulator < -1.0 {
        *accumulator += 1.0;
    } else {
        *accumulator = 0.0;
    }
}

impl CurvatureDrive {
    fn remap_turn(&self, turn: f64) -> f64 {
        let denominator = (FRAC_PI_2 * self.turn_nonlinearity).sin();
        let first_remap = (FRAC_PI_2 * self.turn_nonlinearity * turn).sin() / denominator;
        (FRAC_PI_2 * self.turn_nonlinearity * first_remap) / denominator
    }

    /// Constructs a fresh instance of [`CurvatureDrive`] with the provided constants.
    ///
    /// # Constants
    ///
    /// * `turn_nonlinearity` - Determines how fast the robot's turn traverses a sine curve, and
    ///   affects its nonlinearity
    /// * `deadzone` - Minimum value for `turn` and `throttle` to not ignore and round down to
    ///   zero, creates a deadzone at the center of the joystick
    /// * `slew` - Tunes throttle
    /// * `negative_inertia_scalar` - Used to counteract robot inertia while turning to prevent
    ///   overshooting
    /// * `turn_sensitivity` - Affects sensitivity of turning power, can be used to slow down or
    ///   speed up turning.
    pub fn new(
        turn_nonlinearity: f64,
        deadzone: f64,
        slew: f64,
        negative_inertia_scalar: f64,
        turn_sensitivity: f64,
    ) -> Self {
        Self {
            turn_nonlinearity,
            deadzone,
            slew,
            negative_inertia_scalar,
            turn_sensitivity,

            prev_turn: 0.0,
            prev_throttle: 0.0,
            negative_inertia_accumulator: 0.0,
            quick_stop_accumulator: 0.0,
        }
    }

    /// Runs the Curvature Drive algorithm, updates the internal state, and powers the drivetrain.
    ///
    /// # Examples
    /// ```
    /// struct Robot {
    ///     controller: Controller,
    ///     drivetrain: Differential,
    ///     curvature_drive: CurvatureDrive,
    /// }
    ///
    /// let state = self.controller.state().unwrap();
    /// self.curvature.update(
    ///     &mut self.drivetrain,
    ///     state.left_stick.y(),
    ///     state.right_stick.x(),
    /// ).expect("couldn't set drivetrain voltages");
    /// ```
    pub fn update<T>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        throttle: f64,
        turn: f64,
    ) -> Result<(), MotorError> {
        let mut turn_in_place = false;
        let mut linear_power = throttle;

        if throttle.abs() < self.deadzone && turn.abs() > self.deadzone {
            // deadzone checking
            linear_power = 0.0;
            turn_in_place = true;
        } else if throttle - self.prev_throttle > self.slew {
            linear_power = self.prev_throttle + self.slew;
        } else if throttle - self.prev_throttle < -(self.slew * 2.0) {
            // slew rate is doubled in the opposite direction for faster stopping
            linear_power = self.prev_throttle - (self.slew * 2.0);
        }

        let remapped_turn = self.remap_turn(turn);
        let left;
        let right;

        if turn_in_place {
            // squared for finer control
            left = remapped_turn * remapped_turn.abs();
            right = -remapped_turn * remapped_turn.abs();
        } else {
            let neg_inertia_power = (turn - self.prev_turn) * self.negative_inertia_scalar;
            self.negative_inertia_accumulator += neg_inertia_power;

            let angular_power = linear_power.abs()
                * (remapped_turn + self.negative_inertia_accumulator)
                * self.turn_sensitivity
                - self.quick_stop_accumulator;

            left = linear_power + angular_power;
            right = linear_power - angular_power;

            update_accumulator(&mut self.quick_stop_accumulator);
            update_accumulator(&mut self.negative_inertia_accumulator);
        }

        self.prev_turn = turn;
        self.prev_throttle = throttle;

        drivetrain.motors.set_voltages(Voltages(left, right))
    }
}
