//! Math utilities & types.

#![no_std]

mod angle;
mod vec2;

pub mod curve;

pub use angle::{Angle, IntoAngle};
pub use vec2::Vec2;