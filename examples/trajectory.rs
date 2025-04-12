#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use evian::{
    control::trajectory::{Trajectory, TrajectoryConstraints},
    motion::Ramsete,
    prelude::*,
    shared_motors,
};
use vexide::prelude::*;

#[inline]
#[must_use]
pub const fn from_drive_rpm(rpm: f64, wheel_diameter: f64) -> f64 {
    (rpm / 60.0) * (core::f64::consts::PI * wheel_diameter)
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    const TRACK_WIDTH: f64 = 11.5;
    const DRIVE_RPM: f64 = 450.0;
    const GEARING: f64 = 36.0 / 48.0;
    const WHEEL_DIAMETER: f64 = 3.25;

    let mut imu = InertialSensor::new(peripherals.port_11);
    imu.calibrate().await.unwrap();

    let left_motors = shared_motors![
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
    ];

    let mut drivetrain = Drivetrain::new(
        Differential::from_shared(left_motors.clone(), right_motors.clone()),
        WheeledTracking::forward_only(
            Vec2::default(),
            90.0.deg(),
            [
                TrackingWheel::new(
                    left_motors.clone(),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH / 2.0,
                    Some(GEARING),
                ),
                TrackingWheel::new(
                    right_motors.clone(),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH / 2.0,
                    Some(GEARING),
                ),
            ],
            Some(imu),
        ),
    );

    let constraints = TrajectoryConstraints {
        max_velocity: from_drive_rpm(DRIVE_RPM, WHEEL_DIAMETER),
        max_acceleration: 200.0,
        max_deceleration: 200.0,
        friction_coefficient: 1.0,
        track_width: 11.5,
    };

    let mut ramsete = Ramsete {
        b: 0.00129,
        zeta: 0.2,
        track_width: TRACK_WIDTH,
        wheel_diameter: WHEEL_DIAMETER,
        external_gearing: GEARING,
    };

    let curve = CubicBezier::new((0.0, 0.0), (18.0, 33.0), (50.0, -29.0), (46.0, 8.0));
    let trajectory = Trajectory::generate(curve, 0.1, constraints);

    ramsete.follow(&mut drivetrain, trajectory).await;

    loop {
        println!(
            "{:?} {}",
            drivetrain.tracking.position(),
            drivetrain.tracking.heading().as_degrees()
        );
        sleep(Duration::from_millis(500)).await;
    }
}
