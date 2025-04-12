use vexide::prelude::Float;

/// Constraints for a trapezoidal velocity profile.
#[allow(clippy::struct_field_names)]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalConstraints {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_deceleration: f64,
}

/// Linear, distance-parameterized, 1D trapezoidal motion profile.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalProfile {
    /// Profile Constraints
    constraints: TrapezoidalConstraints,

    /// User-specified velocity at the start of the profile. (d=0)
    initial_velocity: f64,
    /// User-specified velocity at the end of the profile.
    final_velocity: f64,
    /// Velocity during the cruise-phase of the profile.
    cruise_velocity: f64,

    /// Total distance of the profile.
    distance: f64,
    /// Distance of the acceleration phase.
    acceleration_distance: f64,
    /// Distance of the deceleration phase.
    deceleration_distance: f64,
}

impl TrapezoidalProfile {
    #[must_use]
    pub fn new(
        distance: f64,
        initial_velocity: f64,
        final_velocity: f64,
        constraints: TrapezoidalConstraints,
    ) -> Self {
        let max_velocity_squared = constraints.max_velocity * constraints.max_velocity;
        let non_cruise_distance = max_velocity_squared / (2.0 * constraints.max_acceleration)
            + max_velocity_squared / (2.0 * constraints.max_deceleration);

        let cruise_velocity = if non_cruise_distance < distance {
            constraints.max_velocity
        } else {
            (2.0 * (distance * constraints.max_acceleration * constraints.max_deceleration)
                / (constraints.max_acceleration + constraints.max_deceleration))
                .sqrt()
        };
        let cruise_velocity_squared = cruise_velocity * cruise_velocity;

        let acceleration_distance = (cruise_velocity_squared - initial_velocity * initial_velocity)
            / (2.0 * constraints.max_acceleration);
        let deceleration_distance = distance
            + (final_velocity * final_velocity - cruise_velocity_squared)
                / (2.0 * constraints.max_deceleration);

        Self {
            constraints,
            initial_velocity,
            final_velocity,
            cruise_velocity,
            distance,
            acceleration_distance,
            deceleration_distance,
        }
    }

    /// Samples velocity of the profile at a given distance parameter.
    #[must_use]
    pub fn velocity(&self, mut distance: f64) -> f64 {
        distance = distance.clamp(0.0, self.distance);

        if distance < self.acceleration_distance {
            // acceleration phase
            // v = sqrt(vi^2 + 2ad)
            (self.initial_velocity * self.initial_velocity
                + 2.0 * self.constraints.max_acceleration * distance)
                .sqrt()
        } else if distance < self.deceleration_distance {
            // cruise phase, velocity is constant
            self.cruise_velocity
        } else {
            // deceleration phase
            (self.cruise_velocity * self.cruise_velocity
                + 2.0
                    * -self.constraints.max_deceleration
                    * (distance - self.deceleration_distance))
                .sqrt()
        }
    }

    /// Samples acceleration of the profile at a given distance parameter.
    #[must_use]
    pub fn acceleration(&self, mut distance: f64) -> f64 {
        distance = distance.clamp(0.0, self.distance);

        if distance < self.acceleration_distance {
            self.constraints.max_acceleration
        } else if distance < self.deceleration_distance {
            0.0
        } else {
            -self.constraints.max_deceleration
        }
    }
}
