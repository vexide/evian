use alloc::{vec::Vec, sync::Arc};
use core::{
    time::Duration,
    sync::atomic::{Ordering, AtomicBool},
    ops::Drop,
};
use num_traits::real::Real;

use crate::{
    controller::FeedbackController,
    math::{normalize_angle, normalize_motor_power, Vec2},
    tracking::Tracking,
    devices::MotorGroup,
};
#[allow(unused_imports)]
use vex_rt::{
    rtos::{Loop, Mutex, Task, Promise},
    io::*
};

#[derive(Debug, Clone, PartialEq)]
enum DrivetrainTarget {
    Point(Vec2),
    DistanceAndHeading(f64, f64),
}

impl Default for DrivetrainTarget {
    fn default() -> Self {
        Self::DistanceAndHeading(0.0, 0.0)
    }
}

pub struct ThreadedDifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    tracking: T,
    drive_controller: U,
    drive_tolerance: f64,
    drive_error: f64,
    turn_controller: V,
    turn_tolerance: f64,
    turn_error: f64,
    target: DrivetrainTarget,
    settled: bool,
    enabled: bool,
}

pub struct DifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    task: Option<Task>,
    inner: Arc<Mutex<ThreadedDifferentialDrivetrain<T, U, V>>>,
    started: Arc<AtomicBool>,
}

impl<T: Tracking, U: FeedbackController, V: FeedbackController> DifferentialDrivetrain<T, U, V> {
    pub fn new(
        motors: (MotorGroup, MotorGroup),
        tracking: T,
        drive_controller: U,
        turn_controller: V,
        drive_tolerance: f64,
        turn_tolerance: f64,
    ) -> Self {
        let started = Arc::new(AtomicBool::new(false));
        let inner = Arc::new(Mutex::new(ThreadedDifferentialDrivetrain {
            tracking,
            drive_controller,
            turn_controller,
            drive_error: 0.0,
            drive_tolerance,
            turn_tolerance,
            turn_error: 0.0,
            target: DrivetrainTarget::default(),
            settled: false,
            enabled: false,
        }));

        Self {
            inner: Arc::clone(&inner),
            started: Arc::clone(&started),
            task: Some(Task::spawn(move || {
                let sample_rate = Duration::from_millis(10);
                let mut task_loop = Loop::new(sample_rate);

                while started.load(Ordering::Relaxed) {
                    let mut inner = inner.lock();

                    inner.tracking.update();

                    let heading = inner.tracking.heading();
                    let position = inner.tracking.position();
                    let forward_travel = inner.tracking.forward_travel();
    
                    let mut is_point_target = false;
                    let (drive_error, turn_error) = match inner.target {
                        DrivetrainTarget::Point(point) => {
                            is_point_target = true;

                            let displacement = point - position;

                            inner.turn_error = normalize_angle(heading - displacement.angle());
                            inner.drive_error = displacement.length();

                            (inner.drive_error, inner.turn_error)
                        },
                        DrivetrainTarget::DistanceAndHeading(target_distance, target_heading) => {
                            inner.drive_error = target_distance - forward_travel;
                            inner.turn_error = normalize_angle(heading - target_heading);

                            (inner.drive_error, inner.turn_error)
                        },
                    };

                    if !inner.enabled {
                        continue;
                    }

                    let mut drive_output = inner.drive_controller.update(drive_error, sample_rate);
                    let turn_output = inner.turn_controller.update(turn_error, sample_rate);
                    
                    if is_point_target {
                        drive_output *= inner.turn_error.cos();
                    }
                    
                    let (left_voltage, right_voltage) = normalize_motor_power((
                        drive_output + turn_output,
                        drive_output - turn_output,
                    ), 128.0);
    
                    let mut left_motors = motors.0.lock();
                    for motor in left_motors.iter_mut() {
                        motor.move_i8(left_voltage.round() as i8).unwrap();
                    }
    
                    let mut right_motors = motors.1.lock();
                    for motor in right_motors.iter_mut() {
                        motor.move_i8(right_voltage.round() as i8).unwrap();
                    }
    
                    if inner.drive_error < inner.drive_tolerance && (inner.turn_error < inner.turn_tolerance || is_point_target) {
                        inner.settled = true;
                    }
                    
                    task_loop.delay();
                }
            }).unwrap()),
        }
    }

    pub fn enable(&self) {
        let mut inner = self.inner.lock();
        inner.enabled = true;
    }

    pub fn disable(&self) {
        let mut inner = self.inner.lock();
        inner.enabled = false;
    }

    /// Moves the drivetrain in a straight line for a certain distance.
    pub fn drive_distance(&mut self, distance: f64) {
        let mut inner = self.inner.lock();
        inner.settled = false;

        // Add `distance` to the drivetrain's target distance.
        inner.target = DrivetrainTarget::DistanceAndHeading(
            inner.tracking.forward_travel() + distance,
            match inner.target {
                DrivetrainTarget::DistanceAndHeading(_, heading) => heading,
                DrivetrainTarget::Point(_) => inner.tracking.heading(),
            },
        );
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub fn turn_to_angle(&self, angle: f64) {
        let mut inner = self.inner.lock();
        inner.settled = false;

        // Add `distance` to the drivetrain's target distance.
        inner.target = DrivetrainTarget::DistanceAndHeading(
            match inner.target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                DrivetrainTarget::Point(_) => inner.tracking.forward_travel(),
            },
            angle,
        );
    }

    /// Turns the drivetrain in place to face the direction of a certain point.
    pub fn turn_to_point(&self, point: Vec2) { 
        let mut inner = self.inner.lock();
        inner.settled = false;

        let displacement = point - inner.tracking.position();
        
        // Set target heading, keeping the current forward position.
        inner.target = DrivetrainTarget::DistanceAndHeading(
            match inner.target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                DrivetrainTarget::Point(_) => inner.tracking.forward_travel(),
            },
            displacement.angle(),
        );
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub fn move_to_point(&self, point: impl Into<Vec2>) {
        let mut inner = self.inner.lock();
        inner.settled = false;
        
        // Set target heading, keeping the current forward position.
        inner.target = DrivetrainTarget::Point(point.into());
    }

    /// Moves the drivetrain along a path defined by a series of waypoints.
    pub fn follow_path(&self, _path: Vec<Vec2>) {
        todo!();
    }

    // Holds the current angle and position of the drivetrain.
    pub fn hold_position(&self) {
        let mut inner = self.inner.lock();
        inner.target = DrivetrainTarget::DistanceAndHeading(
            inner.tracking.forward_travel(),
            inner.tracking.heading()
        );
    }
}

impl<T: Tracking, U: FeedbackController, V: FeedbackController> Drop for DifferentialDrivetrain<T, U, V> {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            self.started.swap(false, Ordering::Relaxed);

            unsafe {
                task.delete();
            }
        }
    }
}