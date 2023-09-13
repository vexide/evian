#![no_std]
#![no_main]

extern crate alloc;

use alloc::{sync::Arc, vec};
use vex_rt::prelude::*;
use vexnav::prelude::*;

struct Robor {
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<MotorGroup, MotorGroup, InertialSensor>, PIDController, PIDController>,
}

impl Robot for Robor {
    fn new(peripherals: Peripherals) -> Self {
        let m1 = Arc::new(Mutex::new(
            peripherals
                .port01
                .into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false)
                .unwrap(),
        ));
        let m2 = Arc::new(Mutex::new(
            peripherals
                .port02
                .into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false)
                .unwrap(),
        ));
        let m3 = Arc::new(Mutex::new(
            peripherals
                .port03
                .into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false)
                .unwrap(),
        ));
        let m4 = Arc::new(Mutex::new(
            peripherals
                .port04
                .into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false)
                .unwrap(),
        ));

        Self {
            drivetrain: DifferentialDrivetrain::new(
                (vec![m1.clone(), m2.clone()], vec![m3.clone(), m4.clone()]),
                ParallelWheelTracking::new(
                    Vec2::new(0., 0.),
                    90.,
                    TrackingWheel::new(vec![m1.clone(), m2.clone()], 4., Some(84. / 60.)),
                    TrackingWheel::new(vec![m3.clone(), m4.clone()], 4., Some(84. / 60.)),
                    Some(peripherals.port08.into_imu()),
                    14.0,
                ),
                PIDController::new(0., 0., 0.),
                PIDController::new(0., 0., 0.),
                0.3,
                0.3,
            ),
        }
    }
}

entry!(Robor);
