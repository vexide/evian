#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use evian::{
    differential::{commands::basic::BasicMotion, Voltages},
    prelude::*,
};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: DifferentialDrivetrain,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut basic_motion = BasicMotion {
            linear_controller: Pid::new(0.5, 0.0, 0.0, None),
            angular_controller: Pid::new(0.5, 0.0, 0.0, None),
            linear_settler: Settler::new()
                .error_tolerance(0.3)
                .tolerance_duration(Duration::from_millis(100))
                .timeout(Duration::from_secs(2)),
            angular_settler: Settler::new()
                .error_tolerance(0.3)
                .tolerance_duration(Duration::from_millis(100))
                .timeout(Duration::from_secs(2)),
        };

        basic_motion.drive_distance(dt, 10.0).await;
        basic_motion
            .turn_to_heading(dt, Angle::from_degrees(90.0))
            .await;
    }

    async fn driver(&mut self) {
        loop {
            let controller = self.controller.state().unwrap_or_default();

            self.drivetrain.set_voltages((
                controller.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                controller.right_stick.y() * Motor::V5_MAX_VOLTAGE,
            ));

            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = shared_motors![
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
    ];

    Robot {
        controller: peripherals.primary_controller,
        drivetrain: DifferentialDrivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            ParallelWheelTracking::new(
                Vec2::default(),
                Angle::from_degrees(0.0),
                TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                None,
            ),
        ),
    }
    .compete()
    .await;
}
