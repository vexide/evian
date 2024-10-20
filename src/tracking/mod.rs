pub mod parallel_wheel;
pub mod sensor;
pub mod wheel;

use core::fmt::Debug;
use core::prelude::rust_2021::*;

use crate::math::Vec2;

/// A system that performs localization and returns telemetry of a mobile robot.
pub trait Tracking: Send + 'static {
    fn update(&mut self) -> TrackingContext;
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrackingContext {
    pub position: Vec2,
    pub heading: f64,
    pub forward_travel: f64,
    pub linear_velocity: f64,
    pub angular_velocity: f64,
}
