use core::f64::consts::PI;

use vexide::{async_runtime::time::sleep, devices::smart::Motor, prelude::Float};

use crate::{
    differential::{trajectory::Trajectory, Differential, Voltages},
    drivetrain::Drivetrain,
    tracking::{TracksHeading, TracksPosition},
};

/// RAMSETE Unicycle Controller
#[derive(PartialEq)]
pub struct Ramsete {
    pub b: f64,
    pub zeta: f64,
    pub track_width: f64,
    pub wheel_diameter: f64,
    pub external_gearing: f64,
}

#[inline]
pub const fn to_motor_rpm(in_sec: f64, wheel_diameter: f64, external_gearing: f64) -> f64 {
    (in_sec * 60.0) / (PI * wheel_diameter * external_gearing)
}

impl Ramsete {
    pub async fn follow<T: TracksPosition + TracksHeading>(
        &mut self,
        drivetrain: &mut Drivetrain<Differential, T>,
        trajectory: Trajectory,
    ) {
        let mut prev_position = drivetrain.tracking.position();
        let mut distance = 0.0;

        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let position = drivetrain.tracking.position();
            distance += position.distance(prev_position);
            prev_position = position;

            let profile = trajectory.at(distance);

            if *trajectory.profile.last().unwrap() == profile {
                _ = drivetrain.motors.set_voltages(Voltages::default());
                return;
            }

            let desired_linear_velocity = profile.linear_velocity;
            let desired_angular_velocity = profile.angular_velocity;

            // Compute gain value `k`
            let k = 2.0
                * self.zeta
                * ((desired_angular_velocity * desired_angular_velocity)
                    + self.b * (desired_linear_velocity * desired_linear_velocity))
                    .sqrt();

            // Compute error in the local reference frame of the robot (+x is forward)
            let position_error =
                (profile.position - position).rotated(-drivetrain.tracking.heading().as_radians());
            let heading_error = (profile.heading - drivetrain.tracking.heading())
                .wrapped()
                .as_radians();

            // Linear/angular velocity commands
            let angular_velocity = (desired_angular_velocity
                + k * heading_error
                + self.b
                    * desired_linear_velocity
                    * (heading_error.sin() / heading_error)
                    * position_error.y)
                / 2.0
                * self.track_width;
            let linear_velocity =
                desired_linear_velocity * heading_error.cos() + k * position_error.x;

            _ = drivetrain.motors.set_velocities(
                Voltages(
                    to_motor_rpm(
                        linear_velocity - angular_velocity,
                        self.wheel_diameter,
                        self.external_gearing,
                    ),
                    to_motor_rpm(
                        linear_velocity + angular_velocity,
                        self.wheel_diameter,
                        self.external_gearing,
                    ),
                )
                .normalized(600.0),
            );
        }
    }
}
