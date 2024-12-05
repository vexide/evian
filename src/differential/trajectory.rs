//! 2D Trajectory Generation for Differential Drivetrains
//!
//!

use crate::math::{curve::Curve, Angle, Vec2};
use alloc::{vec, vec::Vec};
use vexide::prelude::Float;

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrajectoryConstraints {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_deceleration: f64,
    pub friction_coefficient: f64,
    pub track_width: f64,
}

impl TrajectoryConstraints {
    #[must_use]
    pub fn max_speed(&self, curvature: f64) -> f64 {
        let max_turn_speed = ((2.0 * self.max_velocity / self.track_width) * self.max_velocity)
            / (curvature.abs() * self.max_velocity + (2.0 * self.max_velocity / self.track_width));

        if curvature == 0.0 {
            max_turn_speed
        } else {
            let max_slip_speed =
                (self.friction_coefficient * (1.0 / curvature.abs()) * 9.81 * 39.3701).sqrt();

            max_slip_speed.min(max_turn_speed)
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrajectoryPoint {
    pub linear_velocity: f64,
    pub angular_velocity: f64,
    pub position: Vec2<f64>,
    pub heading: Angle,
    pub distance: f64,
    pub curvature: f64,
}

pub struct Trajectory {
    pub spacing: f64,
    pub profile: Vec<TrajectoryPoint>,
}

impl Trajectory {
    pub fn generate(curve: impl Curve, spacing: f64, constraints: TrajectoryConstraints) -> Self {
        Self {
            spacing,
            profile: {
                let curve = curve;

                let mut t = 0.0;
                let mut distance = 0.0;
                let mut linear_velocity = 0.0;
                let mut previous_angular_velocity = 0.0;

                let mut forward_pass = vec![TrajectoryPoint::default()];
                let mut curvature_cache = Vec::new();

                while t <= curve.max_t() {
                    let derivative = curve.derivative(t);
                    let second_derivative = curve.second_derivative(t);

                    let curvature = {
                        let mut denominator = derivative.dot(derivative);
                        denominator *= denominator.sqrt();
                        derivative.cross(second_derivative) / denominator
                    };
                    curvature_cache.push(curvature);

                    let angular_velocity = linear_velocity * curvature;
                    let angular_accel = (angular_velocity - previous_angular_velocity)
                        * (linear_velocity / spacing);
                    previous_angular_velocity = angular_velocity;

                    let max_acceleration = constraints.max_acceleration
                        - (angular_accel * constraints.track_width / 2.0).abs();
                    linear_velocity = constraints.max_speed(curvature).min(
                        (linear_velocity * linear_velocity + 2.0 * max_acceleration * spacing)
                            .sqrt(),
                    );

                    forward_pass.push(TrajectoryPoint {
                        linear_velocity,
                        angular_velocity: Default::default(), // filled in by reverse pass
                        position: curve.point(t),
                        heading: Angle::from_radians(derivative.y.atan2(derivative.x)),
                        distance,
                        curvature,
                    });

                    t += spacing / derivative.length();
                    distance += spacing;
                }

                let mut reverse_pass: Vec<TrajectoryPoint> = Vec::new();
                let mut i = 0;
                linear_velocity = 0.0;
                previous_angular_velocity = 0.0;

                while i < (forward_pass.len() - 1) {
                    let curvature = *curvature_cache.last().unwrap();
                    curvature_cache.pop();

                    let angular_velocity = linear_velocity * curvature;
                    let angular_acceleration = (angular_velocity - previous_angular_velocity)
                        * (linear_velocity / spacing);
                    previous_angular_velocity = angular_velocity;

                    let max_acceleration = constraints.max_deceleration
                        - (angular_acceleration * constraints.track_width / 2.0).abs();
                    linear_velocity = constraints.max_speed(curvature).min(
                        (linear_velocity * linear_velocity + 2.0 * max_acceleration * spacing)
                            .sqrt(),
                    );

                    let forward_pass_len = forward_pass.len();
                    let mut adjusted = forward_pass[forward_pass_len - i - 1];
                    if linear_velocity < adjusted.linear_velocity {
                        adjusted.linear_velocity = linear_velocity;
                    }
                    adjusted.angular_velocity = adjusted.linear_velocity * curvature;

                    reverse_pass.push(adjusted);

                    i += 1;
                }

                reverse_pass.reverse();
                reverse_pass
            },
        }
    }

    #[must_use]
    pub fn at(&self, d: f64) -> TrajectoryPoint {
        self.profile[((d / self.spacing) as usize).min(self.profile.len() - 1)]
    }
}
