#![no_std]
#![no_main]

extern crate alloc;

use core::cell::RefCell;

use vex_rt::prelude::*;
use vexnav::prelude::*;
use alloc::{sync::Arc, vec, boxed::Box};

struct HelloBot {
    drivetrain: DifferentialDrivetrain
}

impl Robot for HelloBot {
    fn new(peripherals: Peripherals) -> Self {
        let m1 = Arc::new(Mutex::new(peripherals.port01.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));
        let m2 = Arc::new(Mutex::new(peripherals.port02.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));
        let m3 = Arc::new(Mutex::new(peripherals.port03.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));
        let m4 = Arc::new(Mutex::new(peripherals.port04.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));

        HelloBot {
            drivetrain: DifferentialDrivetrain::new(
                (vec![m1.clone(), m2.clone()], vec![m3.clone(), m4.clone()]),
                Arc::new(Mutex::new(ParallelWheelTracking::new(
                    Vec2::new(0., 0.), 90.,
                    TrackingWheel::new(vec![m1.clone(), m2.clone()], 4., Some(84./60.)),
                    TrackingWheel::new(vec![m3.clone(), m4.clone()], 4., Some(84./60.)),
                    Some(peripherals.port08.into_imu()),
                    14.0
                ))),
                Box::new(PIDController::new(0., 0., 0.)),
                Box::new(PIDController::new(0., 0., 0.)),
                0.3, 0.3, 14.
            )
        }
    }
}

entry!(HelloBot);
