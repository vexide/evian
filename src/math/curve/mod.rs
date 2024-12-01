mod bezier;

use crate::math::Vec2;

pub use bezier::CubicBezier;

pub trait Curve {
    fn max_t(&self) -> f64;
    fn point(&self, t: f64) -> Vec2<f64>;
    fn derivative(&self, t: f64) -> Vec2<f64>;
    fn second_derivative(&self, t: f64) -> Vec2<f64>;
}
