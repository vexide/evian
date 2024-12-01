#![no_main]
#![no_std]

extern crate alloc;

use evian::prelude::*;
use vexide::prelude::*;

use core::time::Duration;
use evian::{
    control::pid::{AngularPid, Pid},
    differential::motion::Seeking,
};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut seeking = Seeking {
            distance_controller: Pid::new(0.5, 0.0, 0.0, None),
            angle_controller: AngularPid::new(0.5, 0.0, 0.0, None),
            settler: Settler::new()
                .error_tolerance(0.3)
                .tolerance_duration(Duration::from_millis(100))
                .timeout(Duration::from_secs(2)),
        };

        seeking.move_to_point(dt, (24.0, 24.0)).await;
    }

    async fn driver(&mut self) {
        loop {
            let controller = self.controller.state().unwrap_or_default();

            _ = self.drivetrain.motors.set_voltages((
                controller.left_stick.y() * Motor::V5_MAX_VOLTAGE,
                controller.right_stick.y() * Motor::V5_MAX_VOLTAGE,
            ));

            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            Differential::new(
                shared_motors![
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                ],
                shared_motors![
                    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
                ],
            ),
            ParallelWheelTracking::new(
                Vec2::default(),
                0.0.deg(),
                TrackingWheel::new(
                    RotationSensor::new(peripherals.port_10, Direction::Forward),
                    3.25,
                    7.5,
                    Some(36.0 / 48.0),
                ),
                TrackingWheel::new(
                    RotationSensor::new(peripherals.port_11, Direction::Forward),
                    3.25,
                    7.5,
                    Some(36.0 / 48.0),
                ),
                None,
            ),
        ),
    }
    .compete()
    .await;
}
