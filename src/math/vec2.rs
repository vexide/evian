use core::{fmt, ops};
use num_traits::real::Real;

/// A vector in 2D space.
///
/// The `Vec2` struct represents a two-dimensional vector with x and y components.
#[derive(Clone, PartialEq, Debug, Copy, Default)]
pub struct Vec2 {
    /// The cartesian x coordinate.
    pub x: f64,

    /// The cartesian y coordinate.
    pub y: f64,
}

impl Vec2 {
    /// Construct a `Vec2` from cartesian coordinates.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Get the x component of the vector.
    pub fn x(&self) -> f64 {
        self.x
    }

    /// Get the y component of the vector.
    pub fn y(&self) -> f64 {
        self.y
    }

    /// Construct a `Vec2` from polar coordinates.
    pub fn from_polar(r: f64, theta: f64) -> Self {
        let (sin, cos) = theta.sin_cos();

        Vec2 {
            x: r * cos,
            y: r * sin,
        }
    }

    /// Determine this vector's angle in radians relative to the origin (0, 0).
    pub fn angle(&self) -> f64 {
        self.y.atan2(self.x)
    }

    /// Determine this vector's distance (magnitude) from the origin (0, 0).
    pub fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    /// Compute the dot product between this vector and another `Vec2`.
    ///
    /// The dot product is the sum of the products of each vector's components,
    /// and represents a measurement of how closely two vectors align with respect
    /// to angle.
    pub fn dot(&self, other: &Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Compute the cross product between this vector and another `Vec2`.
    pub fn cross(&self, other: &Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Get the unit (normalized) vector.
    ///
    /// This function creates a `Vec2` with a length of 1.0 while retaining the
    /// angle of its original input.
    pub fn unit(&self) -> Self {
        let magnitude = self.length();

        if magnitude == 0.0 {
            *self
        } else {
            Self {
                x: self.x / magnitude,
                y: self.y / magnitude,
            }
        }
    }

    /// Project one `Vec2` onto onto another.
    pub fn project(&self, onto: &Vec2) -> Self {
        let dot_product = self.dot(onto);
        let onto_mag = onto.length();

        if onto_mag == 0.0 {
            *self
        } else {
            Self {
                x: onto.x * (dot_product / onto_mag.powi(2)),
                y: onto.y * (dot_product / onto_mag.powi(2)),
            }
        }
    }

    /// Create a new vector with its coordinates rotated by a given angle
    /// in radians.
    pub fn rotate(&self, angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();

        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl From<(f64, f64)> for Vec2 {
    fn from(tuple: (f64, f64)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl fmt::Display for Vec2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

impl ops::Add for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl ops::Add<f64> for Vec2 {
    type Output = Self;

    fn add(self, scalar: f64) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl ops::Sub for Vec2 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl ops::Sub<f64> for Vec2 {
    type Output = Self;

    fn sub(self, scalar: f64) -> Self {
        Self {
            x: self.x - scalar,
            y: self.y - scalar,
        }
    }
}

impl ops::Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl ops::Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, scalar: f64) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

impl ops::Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl ops::AddAssign for Vec2 {
    fn add_assign(&mut self, other: Vec2) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl ops::AddAssign<f64> for Vec2 {
    fn add_assign(&mut self, scalar: f64) {
        self.x += scalar;
        self.y += scalar;
    }
}

impl ops::SubAssign for Vec2 {
    fn sub_assign(&mut self, other: Vec2) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl ops::SubAssign<f64> for Vec2 {
    fn sub_assign(&mut self, scalar: f64) {
        self.x -= scalar;
        self.y -= scalar;
    }
}

impl ops::MulAssign<f64> for Vec2 {
    fn mul_assign(&mut self, scalar: f64) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl ops::DivAssign<f64> for Vec2 {
    fn div_assign(&mut self, scalar: f64) {
        self.x /= scalar;
        self.y /= scalar;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::SQRT_2;

    #[test]
    fn angle() {
        assert_eq!(Vec2::new(2.0, 2.0).angle().to_degrees(), 45.0);
        assert_eq!(Vec2::new(0.0, 2.0).angle().to_degrees(), 90.0);
    }

    #[test]
    fn magnitude() {
        assert_eq!(Vec2::new(1.0, 0.0).length(), 1.0);
        assert_eq!(Vec2::new(1.0, 1.0).length(), SQRT_2);
    }

    #[test]
    fn rotation() {
        assert_eq!(
            Vec2::new(1.0, 1.0).rotate(f64::to_radians(45.0)),
            Vec2::new(0.0, SQRT_2)
        );
    }

    #[test]
    fn ops() {
        let vec = Vec2::new(1.0, 2.0);

        // Vector and scalar Add
        assert_eq!(vec + Vec2::new(3.0, 4.0), Vec2::new(4.0, 6.0));
        assert_eq!(vec + 2.0, Vec2::new(3.0, 4.0));

        // Vector and scalar Sub
        assert_eq!(vec - Vec2::new(3.0, 4.0), Vec2::new(-2.0, -2.0));
        assert_eq!(vec - 2.0, Vec2::new(-1.0, 0.0));

        // Vector Mul and Div
        assert_eq!(vec * 2.0, Vec2::new(2.0, 4.0));
        assert_eq!(vec / 2.0, Vec2::new(0.5, 1.0));

        // Vector and scalar AddAssign
        assert_eq!(
            {
                let mut vec = vec.clone();
                vec += 1.0;
                vec
            },
            Vec2::new(2.0, 3.0)
        );
        assert_eq!(
            {
                let mut vec = vec.clone();
                vec += Vec2::new(1.0, 1.0);
                vec
            },
            Vec2::new(2.0, 3.0)
        );

        // Vector and scalar SubAssign
        assert_eq!(
            {
                let mut vec = vec.clone();
                vec -= 1.0;
                vec
            },
            Vec2::new(0.0, 1.0)
        );
        assert_eq!(
            {
                let mut vec = vec.clone();
                vec -= Vec2::new(1.0, 1.0);
                vec
            },
            Vec2::new(0.0, 1.0)
        );

        // TODO(tropix126): Implement MulAssign, DivAssign, Neg
    }
}
