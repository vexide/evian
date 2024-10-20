#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use evian::{
    control::{pid::PidController, settler::Settler},
    differential::{
        commands::basic::BasicMotions,
        drivetrain::{drive_motors, DifferentialDrivetrain, DriveMotors},
    },
    math::Vec2,
    tracking::{parallel_wheel::ParallelWheelTracking, wheel::TrackingWheel},
};
use vexide::prelude::*;

struct Robot {
    pub drivetrain: DifferentialDrivetrain<ParallelWheelTracking<DriveMotors, DriveMotors>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let basic_motions = BasicMotions {
            linear_controller: PidController::new((0.0, 0.0, 0.0), 0.25),
            angular_controller: PidController::new((0.0, 0.0, 0.0), 0.25),
            linear_settler: Settler::new()
                .error_tolerance(0.3)
                .tolerance_time(Duration::from_millis(100))
                .timeout(Duration::from_secs(2)),
            angular_settler: Settler::new()
                .error_tolerance(0.3)
                .tolerance_time(Duration::from_millis(100))
                .timeout(Duration::from_secs(2)),
        };

        self.drivetrain
            .execute(basic_motions.drive_distance(10.0))
            .await;
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = drive_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
    ];
    let right_motors = drive_motors![
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
    ];

    Robot {
        drivetrain: DifferentialDrivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            ParallelWheelTracking::new(
                Vec2::default(),
                90.0,
                TrackingWheel::new(left_motors, 3.25, 7.5, Some(36.0 / 60.0)),
                TrackingWheel::new(right_motors, 3.25, 7.5, Some(36.0 / 60.0)),
                Some(InertialSensor::new(peripherals.port_9)),
            )
        ),
    }
    .compete()
    .await;
}
