#![no_main]
#![no_std]

extern crate alloc;

use core::time::Duration;

use evian::{
    control::{pid::Pid, settler::Settler},
    differential::{
        commands::basic::BasicMotions,
        drivetrain::{drive_motors, DifferentialDrivetrain, DriveMotors},
    },
    math::Vec2,
    tracking::{parallel_wheel::ParallelWheelTracking, wheel::TrackingWheel},
};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<DriveMotors, DriveMotors>>,
    left_motors: DriveMotors,
    right_motors: DriveMotors,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let basic_motions = BasicMotions {
            linear_controller: Pid::new(0.5, 0.0, 0.0, None),
            angular_controller: Pid::new(0.5, 0.0, 0.0, None),
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

    async fn driver(&mut self) {
        loop {
            let left = self.controller.left_stick.y().unwrap_or_default();
            let right = self.controller.right_stick.y().unwrap_or_default();

            for motor in self.left_motors.lock().await.iter_mut() {
                motor.set_voltage(Motor::MAX_VOLTAGE * left).unwrap();
            }
            for motor in self.right_motors.lock().await.iter_mut() {
                motor.set_voltage(Motor::MAX_VOLTAGE * right).unwrap();
            }

            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = drive_motors![
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
    ];
    let right_motors = drive_motors![
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
    ];
    let mut imu = InertialSensor::new(peripherals.port_15);

    imu.calibrate().await.unwrap();

    println!("Calibrated IMU");

    Robot {
        controller: peripherals.primary_controller,
        drivetrain: DifferentialDrivetrain::new(
            left_motors.clone(),
            right_motors.clone(),
            ParallelWheelTracking::new(
                Vec2::default(),
                90.0_f64.to_radians(),
                TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
                Some(imu),
            ),
        ),
        left_motors,
        right_motors,
    }
    .compete()
    .await;
}
