//! Parametric curves.

mod bezier;

use crate::Vec2;

pub use bezier::CubicBezier;

/// Trait describing a parametric curve.
pub trait Curve {
    /// The maximum parameter value in the curve function's domain.
    const MAX_T: f64;

    /// Samples the curve function, returning a 2D point on the curve at the parameter `t`.
    fn point(&self, t: f64) -> Vec2;

    /// Samples the curve's derivative at a given parameter.
    fn derivative(&self, t: f64) -> Vec2;

    /// Samples the curve's second derivative at a given parameter.
    fn second_derivative(&self, t: f64) -> Vec2;
}
