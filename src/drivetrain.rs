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
    devices::ThreadsafeMotorGroup,
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

#[derive(Clone, PartialEq, Debug, Copy, Default)]
pub struct SettleCondition {
    pub error_tolerance: f64,
    pub output_tolerance: f64,
    pub duration: Duration,
    pub timeout: Duration
}

impl SettleCondition {
    pub fn new(error_tolerance: f64, output_tolerance: f64, duration: Duration, timeout: Duration) -> Self {
        Self {
            error_tolerance,
            output_tolerance,
            duration,
            timeout
        }
    }

    pub fn is_settled(&self, error: f64, output: f64, elapsed: Duration) -> bool {
        // if error < self.error_tolerance && output < self.output_tolerance && 
        false // yeah
    }
}

#[derive(Default, Debug)]
pub struct DifferentialDrivetrainState {
    drive_error: f64,
    turn_error: f64,
    drive_settle_condition: SettleCondition,
    turn_settle_condition: SettleCondition,
    target: DrivetrainTarget,
    settled: bool,
}

pub struct DifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    task: Option<Task>,
    tracking: Arc<Mutex<T>>,
    drive_controller: Arc<Mutex<U>>,
    turn_controller: Arc<Mutex<V>>,
    state: Arc<Mutex<DifferentialDrivetrainState>>,
    started: Arc<AtomicBool>,
}

impl<T: Tracking, U: FeedbackController, V: FeedbackController> DifferentialDrivetrain<T, U, V> {
    pub fn new(
        motors: (ThreadsafeMotorGroup, ThreadsafeMotorGroup),
        tracking: T,
        drive_controller: U,
        turn_controller: V,
        drive_settle_condition: SettleCondition,
        turn_settle_condition: SettleCondition,
    ) -> Self {
        let tracking = Arc::new(Mutex::new(tracking));
        let drive_controller = Arc::new(Mutex::new(drive_controller));
        let turn_controller = Arc::new(Mutex::new(turn_controller));
        let state = Arc::new(Mutex::new(DifferentialDrivetrainState {
            drive_settle_condition,
            turn_settle_condition,
            ..Default::default()
        }));
        let started = Arc::new(AtomicBool::new(false));
        
        Self {
            state: Arc::clone(&state),
            tracking: Arc::clone(&tracking),
            drive_controller: Arc::clone(&drive_controller),
            turn_controller: Arc::clone(&turn_controller),
            started: Arc::clone(&started),
            task: Some(Task::spawn(move || {
                let sample_rate = Duration::from_millis(10);
                let mut task_loop = Loop::new(sample_rate);

                while started.load(Ordering::Relaxed) {
                    let mut state = state.lock();
                    let mut tracking = tracking.lock();

                    tracking.update();

                    let heading = tracking.heading();
                    let position = tracking.position();
                    let forward_travel = tracking.forward_travel();

                    drop(tracking);
    
                    let mut is_point_target = false;
                    let (drive_error, turn_error) = match state.target {
                        DrivetrainTarget::Point(point) => {
                            is_point_target = true;

                            let displacement = point - position;

                            state.turn_error = normalize_angle(heading - displacement.angle());
                            state.drive_error = displacement.length();

                            (state.drive_error, state.turn_error)
                        },
                        DrivetrainTarget::DistanceAndHeading(target_distance, target_heading) => {
                            state.drive_error = target_distance - forward_travel;
                            state.turn_error = normalize_angle(heading - target_heading);

                            (state.drive_error, state.turn_error)
                        },
                    };

                    if state.settled {
                        continue;
                    }

                    let mut drive_controller = drive_controller.lock();
                    let mut turn_controller = turn_controller.lock();

                    let mut drive_output = drive_controller.update(drive_error, sample_rate);
                    let turn_output = turn_controller.update(turn_error, sample_rate);
                    
                    drop(drive_controller);
                    drop(turn_controller);

                    if is_point_target {
                        drive_output *= state.turn_error.cos();
                    }
                    
                    let (left_power, right_power) = normalize_motor_power((
                        drive_output + turn_output,
                        drive_output - turn_output,
                    ), 12000.0);
    
                    let mut left_motors = motors.0.lock();
                    for motor in left_motors.iter_mut() {
                        motor.move_voltage((12000.0 * left_power / 100.0).round() as i32).unwrap();
                    }
    
                    let mut right_motors = motors.1.lock();
                    for motor in right_motors.iter_mut() {
                        motor.move_voltage((12000.0 * right_power / 100.0).round() as i32).unwrap();
                    }
    
                    if drive_settle_condition.settled(drive_error, drive_output, 0.0.into()) && (state.turn_error < state.turn_tolerance || is_point_target) {
                        state.settled = true;
                    }
                    
                    task_loop.delay();
                }
            }).unwrap()),
        }
    }

    /// Moves the drivetrain in a straight line for a certain distance.
    pub fn drive_distance(&mut self, distance: f64) {
        let mut state = self.state.lock();
        let tracking = self.tracking.lock();
        state.settled = false;

        // Add `distance` to the drivetrain's target distance.
        state.target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel() + distance,
            match state.target {
                DrivetrainTarget::DistanceAndHeading(_, heading) => heading,
                DrivetrainTarget::Point(_) => tracking.heading(),
            },
        );
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub fn turn_to_angle(&self, angle: f64) {
        let mut state = self.state.lock();
        let tracking = self.tracking.lock();
        state.settled = false;

        // Add `distance` to the drivetrain's target distance.
        state.target = DrivetrainTarget::DistanceAndHeading(
            match state.target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                DrivetrainTarget::Point(_) => tracking.forward_travel(),
            },
            angle,
        );
    }

    /// Turns the drivetrain in place to face the direction of a certain point.
    pub fn turn_to_point(&self, point: Vec2) {
        let mut state = self.state.lock();
        let tracking = self.tracking.lock();
        state.settled = false;

        let displacement = point - tracking.position();
        
        // Set target heading, keeping the current forward position.
        state.target = DrivetrainTarget::DistanceAndHeading(
            match state.target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                DrivetrainTarget::Point(_) => tracking.forward_travel(),
            },
            displacement.angle(),
        );
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub fn move_to_point(&self, point: impl Into<Vec2>) {
        let mut state = self.state.lock();
        state.settled = false;
        
        // Set target heading, keeping the current forward position.
        state.target = DrivetrainTarget::Point(point.into());
    }

    /// Moves the drivetrain along a path defined by a series of waypoints.
    pub fn follow_path(&self, _path: Vec<Vec2>) {
        todo!();
    }

    // Holds the current angle and position of the drivetrain.
    pub fn hold_position(&self) {
        let tracking = self.tracking.lock();

        self.state.lock().target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel(),
            tracking.heading()
        );
    }

    pub fn position(&self) -> Vec2 {
        self.tracking.lock().position()
    }

    pub fn heading(&self) -> f64 {
        self.tracking.lock().heading()
    }

    pub fn forward_travel(&self) -> f64 {
        self.tracking.lock().forward_travel()
    }

    pub fn drive_error(&self) -> f64 {
        self.state.lock().drive_error
    }

    pub fn turn_error(&self) -> f64 {
        self.state.lock().turn_error
    }

    pub fn settled(&self) -> bool {
        self.state.lock().settled
    }

    pub fn tracking(&self) -> Arc<Mutex<T>> {
        Arc::clone(&self.tracking)
    }

    pub fn drive_controller(&self) -> Arc<Mutex<U>> {
        Arc::clone(&self.drive_controller)
    }
    
    pub fn turn_controller(&self) -> Arc<Mutex<V>> {
        Arc::clone(&self.turn_controller)
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