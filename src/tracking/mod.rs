pub mod sensor;
pub mod wheel;
pub mod parallel_wheel;

use core::fmt::Debug;
use core::prelude::rust_2021::*;

use crate::math::Vec2;

/// A system that performs localization and returns telemetry on a mobile robot.
pub trait Tracking: Send + 'static {
    fn forward_travel(&self) -> f64;

    fn heading(&self) -> f64;
    fn set_heading(&mut self, heading: f64);

    fn position(&self) -> Vec2;
    fn set_position(&mut self, position: Vec2);

    fn update(&mut self) -> TrackingContext;
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrackingContext {
    pub position: Vec2,
    pub heading: f64,
    pub forward_travel: f64,
}