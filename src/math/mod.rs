pub mod vec2;

pub use vec2::Vec2;

use core::f64::consts::FRAC_2_PI;

/// Constrain an angle in radians from -π to +π.
///
/// This preserves the angle's direction while keeping it within minimum constrains,
/// allowing certain operations to be performed easier.
pub fn normalize_angle(angle: f64) -> f64 {
    angle % FRAC_2_PI
}