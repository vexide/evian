//! 1D Motion Profiles

/// Functionality for time-parameterized 1D motion profiles.
pub trait MotionProfile {
    /// Samples the profile's velocity at a given time parameter.
    fn velocity(&self, t: f64) -> f64;

    /// Samples the profile's acceleration at a given time parameter.
    fn acceleration(&self, t: f64) -> f64;

    /// Samples the profile's jerk at a given time parameter.
    fn jerk(&self, t: f64) -> f64;
}