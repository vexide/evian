use super::Curve;
use crate::math::Vec2;

pub struct CubicBezier {
    p0: Vec2<f64>,
    p1: Vec2<f64>,
    p2: Vec2<f64>,
    p3: Vec2<f64>,
}

impl CubicBezier {
    pub fn new<P: Into<Vec2<f64>>>(p0: P, p1: P, p2: P, p3: P) -> Self {
        Self {
            p0: p0.into(),
            p1: p1.into(),
            p2: p2.into(),
            p3: p3.into(),
        }
    }
}

impl Curve for CubicBezier {
    fn max_t(&self) -> f64 {
        1.0
    }

    fn point(&self, t: f64) -> Vec2<f64> {
        // polynomial: t^3(p3 + 3(p1 - p2) * p0) + 3t^2(p0 - 2p1 + p2) + 3t(p1 - p0) + p0
        (self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * (t * t * t)
            + (self.p0 - self.p1 * 2.0 + self.p2) * (3.0 * t * t)
            + (self.p1 - self.p0) * (3.0 * t)
            + self.p0
    }

    fn derivative(&self, t: f64) -> Vec2<f64> {
        // polynomial: 3t^2(p3 + 3(p1 - p2) - p0) + 6t(p0 - 2p1 + p2)
        ((self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * (t * t)
            + (self.p0 - self.p1 * 2.0 + self.p2) * (2.0 * t)
            + (self.p1 - self.p0))
            * 3.0
    }

    fn second_derivative(&self, t: f64) -> Vec2<f64> {
        // polynomial: 6t(p3 + 3(p1 - p2) - p0) + 6(p0 - 2p1 + p2)
        ((self.p3 + (self.p1 - self.p2) * 3.0 - self.p0) * t + (self.p0 - self.p1 * 2.0 + self.p2))
            * 6.0
    }
}
