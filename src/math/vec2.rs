use core::{
    fmt,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};
use vexide::{core::float::Float, devices::math::Point2};

/// A vector in 2D space.
///
/// The `Vec2` struct represents a two-dimensional vector with x and y components.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
pub struct Vec2<T> {
    /// The cartesian x coordinate.
    pub x: T,

    /// The cartesian y coordinate.
    pub y: T,
}

impl<T> Vec2<T> {
    /// Construct a `Vec2` from cartesian coordinates.
    pub const fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    /// Sets the vector's x component.
    pub fn set_x(&mut self, x: T) {
        self.x = x;
    }

    /// Sets the vector's y component.
    pub fn set_y(&mut self, y: T) {
        self.y = y;
    }
}

impl<T: Copy> Vec2<T> {
    /// Get the x component of the vector.
    pub const fn x(&self) -> T {
        self.x
    }

    /// Get the y component of the vector.
    pub const fn y(&self) -> T {
        self.y
    }
}

impl<T: Float + Copy + Mul<Output = T>> Vec2<T> {
    /// Construct a `Vec2` from polar coordinates.
    pub fn from_polar(r: T, theta: T) -> Self {
        let (sin, cos) = theta.sin_cos();

        Vec2 {
            x: r * cos,
            y: r * sin,
        }
    }
}

impl<T: Float + Copy> Vec2<T> {
    /// Determine this vector's angle in radians relative to the origin (0, 0).
    pub fn angle(&self) -> T {
        self.y.atan2(self.x)
    }

    /// Determine this vector's distance (magnitude) from the origin (0, 0).
    pub fn length(&self) -> T {
        self.x.hypot(self.y)
    }
}

impl<T: Float + Copy + Sub<Output = T>> Vec2<T> {
    pub fn distance(&self, other: Vec2<T>) -> T {
        (*self - other).length()
    }
}

impl<T: Float + Copy + Div<Output = T>> Vec2<T> {
    /// Get the unit (normalized) vector.
    ///
    /// This function creates a `Vec2` with a length of 1.0 while retaining the
    /// angle of its original input.
    #[must_use]
    pub fn unit(&self) -> Self {
        *self / self.length()
    }
}

impl<T: Float + Copy + Add<Output = T> + Sub<Output = T> + Mul<Output = T>> Vec2<T> {
    /// Linearly interpolate between two vectors.
    #[must_use]
    pub fn lerp(self, other: Vec2<T>, t: T) -> Vec2<T> {
        self + ((other - self) * t)
    }

    /// Create a new vector with its coordinates rotated by a given angle
    /// in radians.
    #[must_use]
    pub fn rotated(&self, angle: T) -> Self {
        let (sin, cos) = angle.sin_cos();

        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl<T: Float + Copy + Mul<Output = T> + Sub<Output = T>> Vec2<T> {
    /// Compute the cross product between this vector and another `Vec2`.
    pub fn cross(&self, other: Vec2<T>) -> T {
        self.x * other.y - self.y * other.x
    }
}

impl<T: Float + Copy + Mul<Output = T> + Add<Output = T>> Vec2<T> {
    /// Compute the dot product between this vector and another `Vec2`.
    ///
    /// The dot product is the sum of the products of each vector's components,
    /// and represents a measurement of how closely two vectors align with respect
    /// to angle.
    pub fn dot(&self, other: Vec2<T>) -> T {
        self.x * other.x + self.y * other.y
    }
}

impl<T: Float + Copy + Mul<Output = T> + Add<Output = T> + Div<Output = T>> Vec2<T> {
    /// Project one `Vec2` onto onto another.
    #[must_use]
    pub fn projected(&self, onto: Vec2<T>) -> Self {
        onto * (self.dot(onto) / onto.length().powi(2))
    }
}

impl<T> From<(T, T)> for Vec2<T> {
    fn from(tuple: (T, T)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl<T> From<Point2<T>> for Vec2<T> {
    fn from(value: Point2<T>) -> Self {
        Self {
            x: value.x,
            y: value.y,
        }
    }
}

impl<T: fmt::Display> fmt::Display for Vec2<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

impl<T: Add<Output = T>> Add for Vec2<T> {
    type Output = Self;

    fn add(self, other: Vec2<T>) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl<T: Add<Output = T> + Copy> Add<T> for Vec2<T> {
    type Output = Self;

    fn add(self, scalar: T) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl<T: Sub<Output = T>> Sub for Vec2<T> {
    type Output = Self;

    fn sub(self, other: Vec2<T>) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl<T: Sub<Output = T> + Copy> Sub<T> for Vec2<T> {
    type Output = Self;

    fn sub(self, scalar: T) -> Self {
        Self {
            x: self.x - scalar,
            y: self.y - scalar,
        }
    }
}

impl<T: Mul<Output = T> + Copy> Mul<T> for Vec2<T> {
    type Output = Self;

    fn mul(self, scalar: T) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl<T: Div<Output = T> + Copy> Div<T> for Vec2<T> {
    type Output = Self;

    fn div(self, scalar: T) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

impl<T: Neg<Output = T>> Neg for Vec2<T> {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl<T: AddAssign> AddAssign for Vec2<T> {
    fn add_assign(&mut self, other: Vec2<T>) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl<T: AddAssign + Copy> AddAssign<T> for Vec2<T> {
    fn add_assign(&mut self, scalar: T) {
        self.x += scalar;
        self.y += scalar;
    }
}

impl<T: SubAssign> SubAssign for Vec2<T> {
    fn sub_assign(&mut self, other: Vec2<T>) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl<T: SubAssign + Copy> SubAssign<T> for Vec2<T> {
    fn sub_assign(&mut self, scalar: T) {
        self.x -= scalar;
        self.y -= scalar;
    }
}

impl<T: MulAssign + Copy> MulAssign<T> for Vec2<T> {
    fn mul_assign(&mut self, scalar: T) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl<T: DivAssign + Copy> DivAssign<T> for Vec2<T> {
    fn div_assign(&mut self, scalar: T) {
        self.x /= scalar;
        self.y /= scalar;
    }
}
