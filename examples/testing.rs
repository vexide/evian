#![no_main]
#![no_std]

extern crate alloc;

use evian::prelude::*;
use vexide::prelude::*;

use core::time::Duration;
use evian::{
    control::{AngularPid, Pid},
    differential::motion::Seeking,
};

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {}
    async fn driver(&mut self) {}
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = shared_motors![
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
    ];

    Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            Differential::new(left_motors.clone(), right_motors.clone()),
            WheeledTracking::forward_only(
                Vec2::default(),
                90.0.deg(),
                [
                    TrackingWheel::new(left_motors, 2.75, -5.75, None),
                    TrackingWheel::new(right_motors, 2.75, 5.25, None),
                ],
                None,
            ),
        ),
    }
    .compete()
    .await;
}
