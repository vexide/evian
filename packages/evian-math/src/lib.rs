//! Math utilities & types.

#![no_std]

mod angle;

pub mod curve;

pub use angle::{Angle, IntoAngle};
pub use glam::DVec2 as Vec2;
