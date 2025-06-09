//! Math utilities & types.

#![no_std]

mod angle;
mod vec2;

pub mod curve;

pub use angle::{Angle, IntoAngle};
pub use vec2::Vec2;

pub fn desaturate<const N: usize>(values: [f64; N], max: f64) -> [f64; N] {
    let largest_magnitude = values.iter().map(|v| v.abs()).fold(0.0, f64::max);

    if largest_magnitude > max {
        values.map(|v| v / largest_magnitude)
    } else {
        values
    }
}