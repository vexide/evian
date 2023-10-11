#![cfg_attr(not(test), no_std)]
#![no_main]

extern crate alloc;

use alloc::{sync::Arc, vec};
use vex_rt::prelude::*;
use vexnav::prelude::*;
use num_traits::real::Real;
use core::time::Duration;

struct Robor {
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<ThreadsafeMotorGroup, ThreadsafeMotorGroup, InertialSensor>, PIDController, PIDController>,
    left_motors: ThreadsafeMotorGroup,
    right_motors: ThreadsafeMotorGroup,
}

impl Robot for Robor {
    fn new(peripherals: Peripherals) -> Self {
        let left_motors = Arc::new(Mutex::new(vec![
            peripherals.port01.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, true).unwrap()
        ]));
        let right_motors = Arc::new(Mutex::new(vec![
            peripherals.port02.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()
        ]));

        Self {
            drivetrain: DifferentialDrivetrain::new(
                (Arc::clone(&left_motors), Arc::clone(&right_motors)),
                ParallelWheelTracking::new(
                    Vec2::new(0.0, 0.0), 90.0.to_radians(),
                    TrackingWheel::new(Arc::clone(&left_motors), 4.0, 6.25, Some(1.0)),
                    TrackingWheel::new(Arc::clone(&right_motors), 4.0, 6.25, Some(1.0)),
                    None
                ),
                PIDController::new(3.0, 0.0, 0.0),
                PIDController::new(3.0, 0.0, 0.0),
                SettleCondition::new(0.3, 0.3, Duration::from_millis(200), Duration::from_secs(20)),
                SettleCondition::new(0.3, 0.3, Duration::from_millis(200), Duration::from_secs(20)),
            ),
            left_motors,
            right_motors
        }
    }

    fn opcontrol(&mut self, _ctx: Context) {
        self.drivetrain.drive_distance(10.0);
    }
}

entry!(Robor);
