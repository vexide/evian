#![no_main]
#![no_std]

use evian::prelude::*;
use vexide::prelude::*;

use vexide::core::time::Instant;

use evian::math::curve::CubicBezier;
use evian::trajectory::{Constraints, Trajectory};

#[vexide::main]
async fn main(_peripherals: Peripherals) {
    let constraints = Constraints {
        max_velocity: 76.0,
        max_acceleration: 200.0,
        max_deceleration: 200.0,
        friction_coefficient: 1.0,
        track_width: 3.25,
    };
    let curve = CubicBezier::new(
        (6.456, -62.624),
        (45.2, -63.999),
        (63.999, -57.809),
        (64.916, -17.002),
    );
    let start = Instant::now();
    let trajectory = Trajectory::generate(curve, 0.1, constraints);
    let elapsed = start.elapsed();
    println!("{:?}", elapsed);
    println!("{:?}", trajectory.at(0.0));
}
