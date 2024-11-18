use core::{
    f64::consts::{FRAC_PI_2, PI},
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Rem, RemAssign, Sub, SubAssign},
};
use vexide::{core::float::Float, devices::position::Position};

#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
    #[inline]
    #[must_use]
    pub const fn from_radians(radians: f64) -> Self {
        Self(radians)
    }

    #[inline]
    #[must_use]
    pub fn from_degrees(degrees: f64) -> Self {
        Self(degrees.to_radians())
    }

    #[inline]
    #[must_use]
    pub fn as_degrees(&self) -> f64 {
        self.0.to_degrees()
    }

    #[inline]
    #[must_use]
    pub const fn as_radians(&self) -> f64 {
        self.0
    }

    #[inline]
    #[must_use]
    pub fn wrapped_half_period(&self) -> Self {
        Self(self.0 % FRAC_PI_2)
    }

    #[inline]
    #[must_use]
    pub fn wrapped_period(&self) -> Self {
        Self(self.0 % PI)
    }

    #[inline]
    #[must_use]
    pub const fn is_sign_positive(self) -> bool {
        self.0.is_sign_positive()
    }

    #[inline]
    #[must_use]
    pub const fn is_sign_negative(self) -> bool {
        self.0.is_sign_negative()
    }

    #[inline]
    #[must_use]
    pub fn min(self, other: Self) -> Self {
        Self(self.0.min(other.0))
    }

    #[inline]
    #[must_use]
    pub fn max(self, other: Self) -> Self {
        Self(self.0.max(other.0))
    }

    #[inline]
    #[must_use]
    pub fn abs(self) -> Self {
        Self(self.0.abs())
    }

    #[inline]
    #[must_use]
    pub fn signum(self) -> f64 {
        self.0.signum()
    }

    #[inline]
    #[must_use]
    pub fn copysign(self, sign: Self) -> Self {
        Self(self.0.copysign(sign.0))
    }

    #[inline]
    #[must_use]
    pub fn mul_add(self, a: Self, b: Self) -> Self {
        Self(self.0.mul_add(a.0, b.0))
    }

    #[inline]
    #[must_use]
    pub fn div_euclid(self, rhs: Self) -> Self {
        Self(self.0.div_euclid(rhs.0))
    }

    #[inline]
    #[must_use]
    pub fn rem_euclid(self, rhs: Self) -> Self {
        Self(self.0.rem_euclid(rhs.0))
    }

    #[inline]
    #[must_use]
    pub fn abs_sub(self, other: Self) -> Self {
        #[allow(deprecated)]
        Self(self.0.abs_sub(other.0))
    }

    #[inline]
    #[must_use]
    pub fn sin(self) -> f64 {
        self.0.sin()
    }

    #[inline]
    #[must_use]
    pub fn cos(self) -> f64 {
        self.0.cos()
    }

    #[inline]
    #[must_use]
    pub fn tan(self) -> f64 {
        self.0.tan()
    }

    #[inline]
    #[must_use]
    pub fn sin_cos(self) -> (f64, f64) {
        (self.sin(), self.cos())
    }

    #[inline]
    #[must_use]
    pub fn sinh(self) -> f64 {
        self.0.sinh()
    }

    #[inline]
    #[must_use]
    pub fn cosh(self) -> f64 {
        self.0.cosh()
    }

    #[inline]
    #[must_use]
    pub fn tanh(self) -> f64 {
        self.0.tanh()
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

impl Rem<Angle> for Angle {
    type Output = Self;

    fn rem(self, rhs: Angle) -> Self::Output {
        Self(self.0 % rhs.0)
    }
}

impl RemAssign for Angle {
    fn rem_assign(&mut self, rhs: Self) {
        self.0 %= rhs.0;
    }
}
