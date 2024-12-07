use core::{
    f64::{
        self,
        consts::{FRAC_PI_2, PI, TAU},
    },
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};
use vexide::{core::float::Float, devices::position::Position};

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Angle(f64);

impl Angle {
    pub const ZERO: Self = Self(0.0);
    pub const QUARTER_TURN: Self = Self(FRAC_PI_2);
    pub const HALF_TURN: Self = Self(PI);
    pub const FULL_TURN: Self = Self(TAU);
    pub const MIN: Self = Self(f64::MIN);
    pub const MAX: Self = Self(f64::MAX);
    pub const EPSILON: Self = Self(f64::EPSILON);

    #[inline]
    #[must_use]
    pub const fn from_radians(radians: f64) -> Self {
        Self(radians)
    }

    #[must_use]
    pub const fn from_gradians(gradians: f64) -> Self {
        Self(gradians * (PI / 180.0))
    }

    #[inline]
    #[must_use]
    pub const fn from_degrees(degrees: f64) -> Self {
        Self(degrees.to_radians())
    }

    #[inline]
    #[must_use]
    pub const fn from_turns(turns: f64) -> Self {
        Self(turns * TAU)
    }

    #[inline]
    #[must_use]
    pub fn asin(y: f64) -> Self {
        Self(y.asin())
    }

    #[inline]
    #[must_use]
    pub fn acos(x: f64) -> Self {
        Self(x.asin())
    }

    #[inline]
    #[must_use]
    pub fn atan(tan: f64) -> Self {
        Self(tan.asin())
    }

    #[inline]
    #[must_use]
    pub fn atan2(y: f64, x: f64) -> Self {
        Self(y.atan2(x))
    }

    #[inline]
    #[must_use]
    pub const fn as_degrees(&self) -> f64 {
        self.0.to_degrees()
    }

    #[inline]
    #[must_use]
    pub fn as_turns(&self) -> f64 {
        self.0 / TAU
    }

    #[inline]
    #[must_use]
    pub const fn as_radians(&self) -> f64 {
        self.0
    }

    #[inline]
    #[must_use]
    pub const fn as_gradians(&self) -> f64 {
        self.0 * (180.0 / PI)
    }

    #[inline]
    #[must_use]
    pub fn wrapped(&self) -> Self {
        Self((-self.0 + PI).rem_euclid(TAU) - PI)
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn abs(self) -> Self {
        Self(self.0.abs())
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn signum(self) -> f64 {
        self.0.signum()
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub const fn copysign(self, sign: Self) -> Self {
        Self(self.0.copysign(sign.0))
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn mul_add(self, a: Self, b: Self) -> Self {
        Self(self.0.mul_add(a.0, b.0))
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn div_euclid(self, rhs: Self) -> Self {
        Self(self.0.div_euclid(rhs.0))
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn abs_sub(self, other: Self) -> Self {
        #[allow(deprecated)]
        Self(self.0.abs_sub(other.0))
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn sin(self) -> f64 {
        self.0.sin()
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn cos(self) -> f64 {
        self.0.cos()
    }

    #[inline]
    #[must_use = "this returns the result of the operation, without modifying the original"]
    pub fn tan(self) -> f64 {
        self.0.tan()
    }

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

pub trait IntoAngle {
    fn deg(self) -> Angle;
    fn grad(self) -> Angle;
    fn rad(self) -> Angle;
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
