#![no_main]
#![no_std]
#![feature(error_in_core)]

extern crate alloc;

use core::error::Error;

use alloc::boxed::Box;
use vexide::prelude::*;
use vexnav::{
    devices::DriveMotors, prelude::*, tracking::{ParallelWheelTracking, TrackingWheel}
};

type AnyError = Box<dyn Error>;

struct Robot {
    pub drivetrain: DifferentialDrivetrain<ParallelWheelTracking<DriveMotors, DriveMotors>>,
    pub controller: Controller,
}

impl CompetitionRobot for Robot {
    type Error = AnyError;

    async fn autonomous(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn driver(&mut self) -> Result<(), Self::Error> {
        loop {
            self.drivetrain
                .execute(self.controller.command(JoystickLayout::Tank)?)
                .await;

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) -> Result<(), AnyError> {
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

    let tracking = ParallelWheelTracking::new(
        Vec2::default(),
        90.0,
        TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 60.0)),
        TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 60.0)),
        Some(InertialSensor::new(peripherals.port_9)),
    );

    Robot {
        drivetrain: DifferentialDrivetrain::new(left_motors, right_motors, tracking),
        controller: peripherals.primary_controller,
    }.compete().await;

    Ok(())
}