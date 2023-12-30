#![cfg_attr(not(test), no_std)]
#![no_main]

extern crate alloc;

use vex_rt::prelude::*;
use vexnav::prelude::*;
use num_traits::real::Real;
use core::time::Duration;

struct Robor {
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<MotorGroup, MotorGroup, InertialSensor>, PIDController, PIDController>,
    controller: Controller,
}

impl Robot for Robor {
    fn new(peripherals: Peripherals) -> Self {
        let left_motors = motor_group![
            peripherals.port01.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true).unwrap(),
            peripherals.port02.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true).unwrap(),
            peripherals.port03.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false).unwrap(),
        ];
        let right_motors = motor_group![
            peripherals.port04.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false).unwrap(),
            peripherals.port05.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, false).unwrap(),
            peripherals.port06.into_motor(Gearset::SixToOne, EncoderUnits::Degrees, true).unwrap(),
        ];

        let tracking = ParallelWheelTracking::new(
            Vec2::new(0.0, 0.0),
            90.0.to_radians(),
            TrackingWheel::new(left_motors.clone(), 3.25, 6.25, Some(36.0 / 60.0)),
            TrackingWheel::new(right_motors.clone(), 3.25, 6.25, Some(36.0 / 60.0)),
            Some(peripherals.port08.into_imu()),
        );

        Self {
            controller: peripherals.master_controller,
            drivetrain: DifferentialDrivetrain::new(
                (left_motors, right_motors),
                tracking,
                PIDController::new((3.0, 0.0, 0.0), 0.3),
                PIDController::new((3.0, 0.0, 0.0), 0.3),
                SettleCondition::new(0.3, 0.3, Duration::from_millis(200), Duration::from_secs(20)),
                SettleCondition::new(0.3, 0.3, Duration::from_millis(200), Duration::from_secs(20)),
                0.3,
            ),
        }
    }

    fn autonomous(&mut self, _ctx: Context) {
        let dt = &mut self.drivetrain;

        dt.drive_distance(10.0);
        dt.wait_until_settled();
        dt.turn_to_angle(90.0.to_degrees());
        dt.wait_until_settled();
    }

    fn opcontrol(&mut self, _ctx: Context) {
        self.drivetrain.control_tank(
            f64::from(self.controller.left_stick.get_y().unwrap()) / 127.0 * 100.0,
            f64::from(self.controller.right_stick.get_y().unwrap()) / 127.0 * 100.0
        );
    }
}

entry!(Robor);
