#![cfg_attr(not(test), no_std)]
#![no_main]

extern crate alloc;

use alloc::{sync::Arc, vec};
use vex_rt::prelude::*;
use vexnav::prelude::*;
use num_traits::real::Real;

struct Robor {
    drivetrain: DifferentialDrivetrain<ParallelWheelTracking<Arc<Mutex<Motor>>, Arc<Mutex<Motor>>, InertialSensor>, PIDController, PIDController>,
}

impl Robot for Robor {
    fn new(peripherals: Peripherals) -> Self {
        let m1 = Arc::new(Mutex::new(peripherals.port01.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));
        let m2 = Arc::new(Mutex::new(peripherals.port02.into_motor(Gearset::EighteenToOne, EncoderUnits::Degrees, false).unwrap()));

        Self {
            drivetrain: DifferentialDrivetrain::new(
                vec![Arc::clone(&m1)], vec![Arc::clone(&m2)],
                ParallelWheelTracking::new(
                    Vec2::new(0.0, 0.0),
                    90.0.to_radians(),
                    TrackingWheel::new(Arc::clone(&m1), 4.0, Some(1.0)),
                    TrackingWheel::new(Arc::clone(&m2), 4.0, Some(1.0)),
                    None,
                    14.0,
                ),
                PIDController::new(5.0, 0.0, 0.0),
                PIDController::new(5.0, 0.0, 0.0),
                0.3,
                0.3,
            ),
        }
    }

    fn autonomous(&mut self, _ctx: Context) {
        self.drivetrain.enable();

        self.drivetrain.drive_distance(10.0);
    }
}

entry!(Robor);
