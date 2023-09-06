use alloc::{sync::Arc, vec::Vec, boxed::Box};
use core::{time::Duration, sync::atomic::Ordering};
use core::ops::Drop;
use num_traits::real::Real;

use crate::{
    controller::FeedbackController,
    math::{normalize_angle, normalize_motor_voltages, Vec2},
    tracking::Tracking,
};
use vex_rt::{
    motor::Motor,
    rtos::{Loop, Mutex, Task},
};

pub struct DifferentialDrivetrain {
    thread: Task,
    motors: (Vec<Arc<Mutex<Motor>>>, Vec<Arc<Mutex<Motor>>>),
    tracking: Arc<Mutex<dyn Tracking>>,
    drive_controller: Box<dyn FeedbackController>,
    turn_controller: Box<dyn FeedbackController>,
    drive_tolerance: f64,
    turn_tolerance: f64,
    lookahead_distance: f64,
    settled: bool,
}

impl DifferentialDrivetrain {
    pub fn new(
        motors: (Vec<Arc<Mutex<Motor>>>, Vec<Arc<Mutex<Motor>>>),
        tracking: Arc<Mutex<dyn Tracking>>,
        drive_controller: Box<dyn FeedbackController>,
        turn_controller: Box<dyn FeedbackController>,
        drive_tolerance: f64,
        turn_tolerance: f64,
        lookahead_distance: f64,
    ) -> Self {
        let tracking_mutex = Arc::clone(&tracking);

        Self {
            motors,
            tracking,
            drive_controller,
            turn_controller,
            drive_tolerance,
            turn_tolerance,
            lookahead_distance,
            settled: false,
            thread: Task::spawn(move || {
                let mut tracking = tracking_mutex.lock();
                let mut task_loop = Loop::new(Duration::from_millis(10));

                loop {
                    tracking.update();
                    task_loop.delay();
                }
            }).unwrap()
        }
    }

    /// Moves the drivetrain in a straight line for a certain distance, then stops and holds
    /// the final position once in tolerance.
    pub async fn drive_distance(&mut self, distance: f64) {
        for m in self.motors.0.iter() {
        }
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub async fn turn_to_angle(&self, angle: f64) {
    }

    /// Turns the drivetrain in place to face a certain point.
    pub async fn turn_to_point(&self, point: Vec2) {
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub async fn move_to_point(&self, point: Vec2) {
    }

    /// Moves the drivertain along a path defined by a series of waypoints.
    pub async fn follow_path(&self, path: Vec<Vec2>) {
    }

    // Holds the current angle and position of the drivetrain.
    pub async fn hold_position(&self) {
    }
}

impl Drop for DifferentialDrivetrain {
    fn drop(&mut self) {
        unsafe {
            self.thread.delete();
        }
    }
}