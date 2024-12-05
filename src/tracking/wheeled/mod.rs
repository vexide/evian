mod parallel;
mod perpendicular;
mod wheel;

pub use parallel::ParallelWheelTracking;
pub use perpendicular::PerpendicularWheelTracking;
pub use wheel::TrackingWheel;

use crate::math::{Angle, Vec2};

/// Generic tracking data returned by [`ParallelWheelTracking`] and [`PerpendicularWheelTracking`].
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    position: Vec2<f64>,
    heading: Angle,
    heading_offset: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}
