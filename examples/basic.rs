#![no_main]
#![no_std]

extern crate alloc;

use vexide::prelude::*;

use core::time::Duration;
use evian::{
    control::loops::{AngularPid, Pid},
    motion::{Basic, Seeking},
    prelude::*,
};

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(4.0)
    .velocity(0.25)
    .duration(Duration::from_millis(15));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(f64::to_radians(8.0))
    .velocity(0.09)
    .duration(Duration::from_millis(15));

struct Robot {
    drivetrain: Drivetrain<Differential, WheeledTracking>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut seeking = Seeking {
            linear_controller: LINEAR_PID,
            angular_controller: ANGULAR_PID,
            tolerances: LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        let mut basic = Basic {
            linear_controller: LINEAR_PID,
            angular_controller: ANGULAR_PID,
            linear_tolerances: LINEAR_TOLERANCES,
            angular_tolerances: ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        // Drive forwards at 60% speed.
        basic
            .drive_distance(dt, 24.0)
            .with_linear_output_limit(Motor::V5_MAX_VOLTAGE * 0.6)
            .await;

        // Turn to 0 degrees heading.
        basic.turn_to_heading(dt, 0.0.deg()).await;

        // Move to point (24, 24) on the field.
        seeking.move_to_point(dt, (24.0, 24.0)).await;

        // Having fun with modifiers.
        basic
            .drive_distance_at_heading(dt, 8.0, 45.0.deg())
            .with_linear_kd(1.2)
            .with_angular_tolerance_duration(Duration::from_millis(5))
            .with_angular_error_tolerance(f64::to_radians(10.0))
            .with_linear_error_tolerance(12.0)
            .await;
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = shared_motors![
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
    ];

    let mut basic = Basic {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
        timeout: Some(Duration::from_secs(10)),
    };

    let mut drivetrain = Drivetrain::new(
        Differential::from_shared(left_motors.clone(), right_motors.clone()),
        WheeledTracking::forward_only(
            Vec2::default(),
            90.0.deg(),
            [
                TrackingWheel::new(left_motors, 2.75, -5.75, None),
                TrackingWheel::new(right_motors, 2.75, 5.25, None),
            ],
            None,
        ),
    );

    basic.turn_to_heading(&mut drivetrain, 0.0.deg()).await;

    Robot { drivetrain }.compete().await;
}
