use crate::math::Vec2;

pub mod bezier;
pub mod trajectory;

pub trait TrajectoryCurve {
    fn max_t(&self) -> f64;
    fn point(&self, t: f64) -> Vec2<f64>;
    fn derivative(&self, t: f64) -> Vec2<f64>;
    fn second_derivative(&self, t: f64) -> Vec2<f64>;
}