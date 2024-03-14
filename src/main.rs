#![no_main]
#![no_std]

extern crate alloc;

use pros::prelude::*;
use vexnav::{
    devices::DriveMotors, drivetrain::Voltages, prelude::*, tracking::{ParallelWheelTracking, TrackingWheel}
};

/// Store robot state that will be used throughout the program.
struct VexRobot {
    controller: Controller,
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<DriveMotors, DriveMotors>>,
}

impl VexRobot {
    /// Create subsystems and set up anything that will be used throughout the program.
    /// This function is run by PROS as soon as the robot program is selected.
    pub fn new(peripherals: Peripherals) -> Self {
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

        Self {
            drivetrain: DifferentialDrivetrain::new(left_motors, right_motors, tracking),
            controller: Controller::Master,
        }
    }
}

impl AsyncRobot for VexRobot {
    /// Runs when the robot is enabled in autonomous mode.
    async fn auto(&mut self) -> Result {
        self.drivetrain.execute(Voltages(12.0, 12.0));

        Ok(())
    }

    /// Runs when the robot is enabled in driver control mode.
    async fn opcontrol(&mut self) -> Result {
        loop {
            _ = self.drivetrain.execute(
                self.controller.command(JoystickLayout::Tank)?
            );

            sleep(Motor::DATA_READ_RATE).await;
        }
    }
}

// Register the robot with PROS so that its methods will be called.
async_robot!(
    VexRobot,
    VexRobot::new(Peripherals::take().unwrap_or_else(|| unsafe {
        println!("who yoinked my Peripherals??");
        Peripherals::steal() // gimme that
    }))
);
