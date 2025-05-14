//! Cheesy Drive (aka Curvature drive)
//!
//! This module provides the Cheesy drivetrain controller through [`Cheesy`].
//! Its implemenation is based on <https://wiki.purduesigbots.com/software/robotics-basics/curvature-cheesy-drive>.
//!
//! # Algorithm
//!
//! Cheesy Drive is a nonlinear and curvature-based drivetrain control algorithm. Optimized for
//! driver intuition and precise handling, it smooths inputs and adapts to the situation. Unlike
//! other algorithms such as Arcade Drive and Tank Drive, it performs some mathematical computations
//! and accepts some constants and maintains an internal state that changes every time the algorithm
//! is ran using [`Cheesy::update`].

use core::f64::consts::FRAC_PI_2;
use vexide::float::Float;

use super::Voltages;

/// Cheesy drive controller. This maintains internal state, so you need a mutable reference to
/// use it with the [`Cheesy::update`] method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cheesy {
    turn_nonlinearity: f64,
    deadzone: f64,
    slew: f64,
    negative_inertia_scalar: f64,
    turn_sensitivity: f64,

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

impl Cheesy {
    fn remap_turn(&self, turn: f64) -> f64 {
        let denominator = (FRAC_PI_2 * self.turn_nonlinearity).sin();
        let first_remap = (FRAC_PI_2 * self.turn_nonlinearity * turn).sin() / denominator;
        (FRAC_PI_2 * self.turn_nonlinearity * first_remap) / denominator
    }

    /// Constructs a fresh instance of [`Cheesy`] with the provided constants.
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

    /// Runs the Cheesy Drive algorithm, updates the internal state, and returns [`Voltages`] to be
    /// used to power a [`super::Differential`] drivetrain.
    ///
    /// # Examples
    /// ```
    /// struct Robot {
    ///     controller: Controller,
    ///     drivetrain: Differential,
    ///     cheesy: Cheesy,
    /// }
    ///
    /// let state = self.controller.state.expect("couldn't read controller");
    /// let voltages = self.cheesy.update(state.left_stick.y(), state.right_stick.x());
    /// self.drivetrain.set_voltages(voltages).expect("couldn't set drivetrain voltages");
    /// ```
    pub fn update(&mut self, throttle: f64, turn: f64) -> Voltages {
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

        Voltages(left, right)
    }
}
