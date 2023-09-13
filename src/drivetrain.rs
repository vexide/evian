use alloc::{sync::Arc, vec::Vec, boxed::Box};
use core::sync::atomic::AtomicBool;
use core::{time::Duration, sync::atomic::Ordering};
use core::ops::Drop;
use num_traits::real::Real;

use crate::{
    controller::FeedbackController,
    math::{normalize_angle, normalize_motor_voltages, Vec2},
    tracking::Tracking,
    devices::MotorGroup,
};
use vex_rt::{
    motor::Motor,
    rtos::{Loop, Mutex, Task},
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

pub struct DifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    task: Option<Task>,
    _motors: Arc<(MotorGroup, MotorGroup)>,
    tracking: Arc<Mutex<T>>,
    drive_controller: Arc<Mutex<U>>,
    turn_controller: Arc<Mutex<V>>,
    drive_tolerance: Arc<f64>,
    turn_tolerance: Arc<f64>,
    target: Arc<Mutex<DrivetrainTarget>>,
    settled: Arc<AtomicBool>,
    enabled: Arc<AtomicBool>,
    started: Arc<AtomicBool>
    // lookahead_distance: f64,
}

impl<T: Tracking, U: FeedbackController, V: FeedbackController> DifferentialDrivetrain<T, U, V> {
    pub fn new(
        motors: (MotorGroup, MotorGroup),
        tracking: T,
        drive_controller: U,
        turn_controller: V,
        drive_tolerance: f64,
        turn_tolerance: f64,
        // lookahead_distance: f64,
    ) -> Self {
        let motors = Arc::new(motors);
        let drive_controller = Arc::new(Mutex::new(drive_controller));
        let turn_controller = Arc::new(Mutex::new(turn_controller));
        let tracking = Arc::new(Mutex::new(tracking));
        let drive_tolerance = Arc::new(drive_tolerance);
        let turn_tolerance = Arc::new(turn_tolerance);
        let target = Arc::new(Mutex::new(DrivetrainTarget::default()));
        let settled = Arc::new(AtomicBool::new(false));
        let enabled = Arc::new(AtomicBool::new(false));
        let started = Arc::new(AtomicBool::new(false));

        Self {
            _motors: Arc::clone(&motors),
            tracking: Arc::clone(&tracking),
            drive_controller: Arc::clone(&drive_controller),
            turn_controller: Arc::clone(&turn_controller),
            drive_tolerance: Arc::clone(&drive_tolerance),
            turn_tolerance: Arc::clone(&turn_tolerance),
            // lookahead_distance,
            settled: Arc::clone(&settled),
            enabled: Arc::clone(&enabled),
            started: Arc::clone(&started),
            target: Arc::clone(&target),
            task: Some(Task::spawn(move || {
                let mut tracking = tracking.lock();
                let mut drive_controller = drive_controller.lock();
                let mut turn_controller = turn_controller.lock();
                let target = target.lock();

                let mut task_loop = Loop::new(Duration::from_millis(10));
    
                while started.load(Ordering::Relaxed) {
                    tracking.update();
    
                    let (drive_error, turn_error) = match *target {
                        DrivetrainTarget::Point(point) => {
                            let local_target = point - tracking.position();
                            let turn_error =
                                normalize_angle(tracking.heading() - local_target.angle());
                            let drive_error = local_target.length() * turn_error.cos();
    
                            (drive_error, turn_error)
                        },
                        DrivetrainTarget::DistanceAndHeading(distance, heading) => (
                            distance - tracking.forward_travel(),
                            normalize_angle(heading - tracking.heading()),
                        ),
                    };
    
                    let drive_output = drive_controller.update(drive_error);
                    let turn_output = turn_controller.update(turn_error);
                    
                    let (left_voltage, right_voltage) = normalize_motor_voltages((
                        drive_output + turn_output,
                        drive_output - turn_output,
                    ), 128.0);
    
                    for motor in motors.0.iter() {
                        let mut motor = motor.lock();
                        motor.move_voltage(left_voltage.round() as i32).unwrap();
                    }
    
                    for motor in motors.1.iter() {
                        let mut motor = motor.lock();
                        motor.move_voltage(right_voltage.round() as i32).unwrap();
                    }
    
                    if drive_error < *drive_tolerance && turn_error < *turn_tolerance {
                        settled.swap(true, Ordering::Relaxed);
                    }
    
                    task_loop.delay();
                }
            }).unwrap()),
        }
    }

    pub fn enable(&self) {
        self.enabled.swap(true, Ordering::Relaxed);
    }

    pub fn disable(&self) {
        self.enabled.swap(false, Ordering::Relaxed);
    }

    /// Moves the drivetrain in a straight line for a certain distance.
    pub async fn drive_distance(&mut self, distance: f64) {
        self.settled.swap(false, Ordering::Relaxed);

        let target = Arc::clone(&self.target);
        let tracking = Arc::clone(&self.tracking);
        let mut target = target.lock();
        let tracking = tracking.lock();
        
        // Add `distance` to the drivetrain's target distance.
        *target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel() + distance,
            tracking.heading(),
        );
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub async fn turn_to_angle(&self, angle: f64) {
        self.settled.swap(false, Ordering::Relaxed);

        let target = Arc::clone(&self.target);
        let tracking = Arc::clone(&self.tracking);
        let mut target = target.lock();
        let tracking = tracking.lock();
        
        // Set target heading, keeping the current forward position.
        *target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel(),
            angle,
        );
    }

    /// Turns the drivetrain in place to face the direction of a certain point.
    pub async fn turn_to_point(&self, point: Vec2) { 
        self.settled.swap(false, Ordering::Relaxed);
           
        let target = Arc::clone(&self.target);
        let tracking = Arc::clone(&self.tracking);
        let mut target = target.lock();
        let tracking = tracking.lock();

        let displacement = point - tracking.position();
        
        // Set target heading, keeping the current forward position.
        *target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel(),
            displacement.angle(),
        );
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub async fn move_to_point(&self, point: Vec2) {
        self.settled.swap(false, Ordering::Relaxed);
       
        let target = Arc::clone(&self.target);
        let mut target = target.lock();
        
        // Set target heading, keeping the current forward position.
        *target = DrivetrainTarget::Point(point);
    }

    /// Moves the drivertain along a path defined by a series of waypoints.
    pub async fn follow_path(&self, _path: Vec<Vec2>) {
        todo!();
    }

    // Holds the current angle and position of the drivetrain.
    pub async fn hold_position(&self) {
        self.settled.swap(false, Ordering::Relaxed);

        let target = Arc::clone(&self.target);
        let tracking = Arc::clone(&self.tracking);
        let mut target = target.lock();
        let tracking = tracking.lock();
        
        // Add `distance` to the drivetrain's target distance.
        *target = DrivetrainTarget::DistanceAndHeading(
            tracking.forward_travel(),
            tracking.heading(),
        );
    }
}

impl<T: Tracking, U: FeedbackController, V: FeedbackController> Drop for DifferentialDrivetrain<T, U, V> {
    fn drop(&mut self) {
        if let Some(task) = &self.task {
            self.started.swap(false, Ordering::Relaxed);

            unsafe {
                task.delete();
            }
        }
    }
}