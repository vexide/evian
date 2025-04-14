use core::{
    f64::{
        self,
        consts::{FRAC_PI_2, PI, TAU},
    },
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};
use vexide::{devices::position::Position, float::Float};

/// Angular position.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Angle(f64);

impl Angle {
    /// Angle representing zero rotation.
    pub const ZERO: Self = Self(0.0);

    /// Angle representing a quarter turn around a full circle.
    pub const QUARTER_TURN: Self = Self(FRAC_PI_2);

    /// Angle representing a half turn around a full circle.
    pub const HALF_TURN: Self = Self(PI);

    /// Angle representing a full turn around a circle.
    pub const FULL_TURN: Self = Self(TAU);

    /// Smallest value that can be represented by the `Angle` type.
    /// 
    /// Equivalent to [`f64::MIN`] radians.
    pub const MIN: Self = Self(f64::MIN);

    /// Largest value that can be represented by the `Angle` type.
    /// 
    /// Equivalent to [`f64::MAX`] radians.
    pub const MAX: Self = Self(f64::MAX);

    /// [Machine epsilon] value for `Angle`.
    /// 
    /// [Machine epsilon]: https://en.wikipedia.org/wiki/Machine_epsilon
    pub const EPSILON: Self = Self(f64::EPSILON);

    /// Creates a new `Angle` from a value in radians.
    #[inline]
    #[must_use]
    pub const fn from_radians(radians: f64) -> Self {
        Self(radians)
    }

    /// Creates a new `Angle` from a value in gradians.
    #[must_use]
    pub const fn from_gradians(gradians: f64) -> Self {
        Self(gradians * (PI / 200.0))
    }

    /// Creates a new `Angle` from a value in degrees.
    #[inline]
    #[must_use]
    pub const fn from_degrees(degrees: f64) -> Self {
        Self(degrees.to_radians())
    }

    /// Creates a new `Angle` from a value in turns (revolutions).
    #[inline]
    #[must_use]
    pub const fn from_turns(turns: f64) -> Self {
        Self(turns * TAU)
    }

    /// Computes the arcsine of a number. Return value is in the range
    /// [-pi/2, pi/2] or NaN if the angle is outside the range [-1, 1].
    #[inline]
    #[must_use]
    pub fn asin(y: f64) -> Self {
        Self(y.asin())
    }

    /// Computes the arccosine of a number. Return value is in the range
    /// [0, pi] or NaN if the number is outside the range [-1, 1].
    #[inline]
    #[must_use]
    pub fn acos(x: f64) -> Self {
        Self(x.acos())
    }

    /// Computes the arctangent of an angle. Return value is in radians in the
    /// range [-pi/2, pi/2];
    #[inline]
    #[must_use]
    pub fn atan(tan: f64) -> Self {
        Self(tan.atan())
    }

    /// Computes the four quadrant arctangent angle of `y` and `x`.
    #[inline]
    #[must_use]
    pub fn atan2(y: f64, x: f64) -> Self {
        Self(y.atan2(x))
    }

    /// Returns this angle's value in degrees.
    #[inline]
    #[must_use]
    pub const fn as_degrees(&self) -> f64 {
        self.0.to_degrees()
    }

    /// Returns this angle's value in turns (revolution).
    #[inline]
    #[must_use]
    pub fn as_turns(&self) -> f64 {
        self.0 / TAU
    }

    /// Returns this angle's value in radians.
    #[inline]
    #[must_use]
    pub const fn as_radians(&self) -> f64 {
        self.0
    }

    /// Returns this angle's value in gradians.
    #[inline]
    #[must_use]
    pub const fn as_gradians(&self) -> f64 {
        self.0 * (200.0 / PI)
    }

    /// Normalizes an angle to the bounds [-pi, pi].
    #[inline]
    #[must_use]
    pub fn wrapped(&self) -> Self {
        Self((-self.0 + PI).rem_euclid(TAU) - PI)
    }

    /// Normalizes an angle to the bounds [0, 2pi].
    #[inline]
    #[must_use]
    pub fn wrapped_positive(&self) -> Self {
        Self(self.0.rem_euclid(TAU))
    }

    /// Computes the absolute value of `self`.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn abs(self) -> Self {
        Self(self.0.abs())
    }

    /// Returns a number that represents the sign of `self`.
    ///
    /// - `1.0` if the number is positive, `+0.0` or `INFINITY`
    /// - `-1.0` if the number is negative, `-0.0` or `NEG_INFINITY`
    /// - NaN if the number is NaN
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn signum(self) -> f64 {
        self.0.signum()
    }

    /// Returns an angle composed of the magnitude of `self` and the sign of
    /// `sign`.
    ///
    /// Equal to `self` if the sign of `self` and `sign` are the same, otherwise equal to `-self`.
    /// If `self` is a NaN, then a NaN with the same payload as `self` and the sign bit of `sign` is
    /// returned.
    ///
    /// If `sign` is a NaN, then this operation will still carry over its sign into the result. Note
    /// that IEEE 754 doesn't assign any meaning to the sign bit in case of a NaN, and as Rust
    /// doesn't guarantee that the bit pattern of NaNs are conserved over arithmetic operations, the
    /// result of `copysign` with `sign` being a NaN might produce an unexpected or non-portable
    /// result. See the [specification of NaN bit patterns](primitive@f32#nan-bit-patterns) for more
    /// info.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn copysign(self, sign: Self) -> Self {
        Self(self.0.copysign(sign.0))
    }

    /// Fused multiply-add. Computes `(self * a) + b` with only one rounding
    /// error, yielding a more accurate result than an unfused multiply-add.
    ///
    /// Using `mul_add` *may* be more performant than an unfused multiply-add if
    /// the target architecture has a dedicated `fma` CPU instruction. However,
    /// this is not always true, and will be heavily dependant on designing
    /// algorithms with specific target hardware in mind.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn mul_add(self, a: Self, b: Self) -> Self {
        Self(self.0.mul_add(a.0, b.0))
    }

    /// Calculates Euclidean division.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn div_euclid(self, rhs: Self) -> Self {
        Self(self.0.div_euclid(rhs.0))
    }

    /// The positive difference of two numbers.
    ///
    /// * If `self <= other`: `0.0`
    /// * Else: `self - other`
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn abs_sub(self, other: Self) -> Self {
        #[allow(deprecated)]
        Self(self.0.abs_sub(other.0))
    }

    /// Computes the sine of an angle.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn sin(self) -> f64 {
        self.0.sin()
    }

    /// Computes the cosine of an angle.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn cos(self) -> f64 {
        self.0.cos()
    }

    /// Computes the tangent of an angle.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn tan(self) -> f64 {
        self.0.tan()
    }

    /// Simultaneously computes the sine and cosine of the number, `x`. Returns
    /// `(sin(x), cos(x))`.
    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn sin_cos(self) -> (f64, f64) {
        self.0.sin_cos()
    }
}

impl From<Position> for Angle {
    fn from(value: Position) -> Self {
        Self::from_degrees(value.as_degrees())
    }
}

impl Add<Angle> for Angle {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub<Angle> for Angle {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Mul<f64> for Angle {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for Angle {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl AddAssign<Angle> for Angle {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl SubAssign<Angle> for Angle {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl MulAssign<f64> for Angle {
    #[inline]
    fn mul_assign(&mut self, rhs: f64) {
        self.0 *= rhs;
    }
}

impl DivAssign<f64> for Angle {
    #[inline]
    fn div_assign(&mut self, rhs: f64) {
        self.0 /= rhs;
    }
}

impl Neg for Angle {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

/// Extension trait for easily creating [`Angle`]s from floating-point
/// number literals.
pub trait IntoAngle {
    /// Creates an [`Angle`] of `self` degrees.
    fn deg(self) -> Angle;

    /// Creates an [`Angle`] of `self` gradians.
    fn grad(self) -> Angle;

    /// Creates an [`Angle`] of `self` radians.
    fn rad(self) -> Angle;

    /// Creates an [`Angle`] of `self` turns (revolutions).
    fn turns(self) -> Angle;
}

impl IntoAngle for f64 {
    fn deg(self) -> Angle {
        Angle::from_degrees(self)
    }

    fn rad(self) -> Angle {
        Angle::from_radians(self)
    }

    fn grad(self) -> Angle {
        Angle::from_gradians(self)
    }

    fn turns(self) -> Angle {
        Angle::from_turns(self)
    }
}
