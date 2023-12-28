use alloc::{vec::Vec, sync::Arc};
use core::{
    time::Duration,
    sync::atomic::{Ordering, AtomicBool},
    ops::Drop,
};
use num_traits::real::Real;
use vex_rt::rtos::{Loop, Mutex, Task, Instant};

#[allow(unused_imports)]
use vex_rt::io::*;

use crate::{
    controller::FeedbackController,
    math::{normalize_angle, normalize_motor_power, Vec2, LineCircleIntersections},
    tracking::Tracking,
    devices::MotorGroup,
    timer::Timer,
};

#[derive(Debug, Clone, Copy, PartialEq)]
enum DrivetrainTarget {
    Point(Vec2),
    DistanceAndHeading(f64, f64),
    MotorPower(f64, f64),
}

impl Default for DrivetrainTarget {
    fn default() -> Self {
        DrivetrainTarget::MotorPower(0.0, 0.0)
    }
}

#[derive(Clone, PartialEq, Debug, Copy)]
pub struct SettleCondition {
    pub error_tolerance: f64,
    pub output_tolerance: f64,
    pub duration: Duration,
    pub timeout: Duration,
    timeout_timestamp: Instant,
    timestamp: Instant,
}

impl Default for SettleCondition {
    fn default() -> Self {
        Self {
            error_tolerance: f64::default(),
            output_tolerance: f64::default(),
            duration: Duration::default(),
            timeout: Duration::default(),
            timeout_timestamp: Instant::now(),
            timestamp: Instant::now(),
        }
    }
}

impl SettleCondition {
    pub fn new(error_tolerance: f64, output_tolerance: f64, duration: Duration, timeout: Duration) -> Self {
        Self {
            error_tolerance,
            output_tolerance,
            duration,
            timeout,
            timestamp: Instant::now(),
            timeout_timestamp: Instant::now(),
        }
    }
    
    pub fn is_settled(&mut self, error: f64, output: f64) -> bool {
        if error > self.error_tolerance || output > self.output_tolerance {
            self.timestamp = Instant::now();
        }
        
        self.timestamp.elapsed() > self.duration || self.timeout_timestamp.elapsed() > self.timeout
    }

    pub fn reset(&mut self) {
        self.timestamp = Instant::now();
        self.timeout_timestamp = Instant::now();
    }
}

#[derive(Default, Debug)]
pub struct DifferentialDrivetrainState {
    drive_error: f64,
    turn_error: f64,
    drive_settle_condition: SettleCondition,
    turn_settle_condition: SettleCondition,
    target: DrivetrainTarget,
    lookahead_distance: f64,
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
        motors: (MotorGroup, MotorGroup),
        tracking: T,
        drive_controller: U,
        turn_controller: V,
        drive_settle_condition: SettleCondition,
        turn_settle_condition: SettleCondition,
        lookahead_distance: f64,
    ) -> Self {
        let tracking = Arc::new(Mutex::new(tracking));
        let drive_controller = Arc::new(Mutex::new(drive_controller));
        let turn_controller = Arc::new(Mutex::new(turn_controller));
        let state = Arc::new(Mutex::new(DifferentialDrivetrainState {
            drive_settle_condition,
            turn_settle_condition,
            lookahead_distance,
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
                const SAMPLE_RATE: Duration = Duration::from_millis(10);
                let mut task_loop = Loop::new(SAMPLE_RATE);

                while started.load(Ordering::Relaxed) {
                    let mut tracking = tracking.lock();

                    // Get updated tracking data
                    tracking.update();
                    let heading = tracking.heading();
                    let position = tracking.position();
                    let forward_travel = tracking.forward_travel();
                    drop(tracking);

                    println!("{:?}", position); // debug
                    
                    let mut state = state.lock();

                    // Calculate left and right wheel power based on target type.
                    let (left_power, right_power) = match state.target {
                        DrivetrainTarget::Point(point) => {
                            let displacement = point - position; // Displacement vector from our current position to the target

                            state.turn_error = normalize_angle(heading - displacement.angle());
                            state.drive_error = displacement.length();

                            let drive_power = drive_controller.lock().update(state.drive_error, SAMPLE_RATE) * state.turn_error.cos();
                            let turn_power = turn_controller.lock().update(state.turn_error, SAMPLE_RATE);

                            let drive_error = state.drive_error;
                            if state.drive_settle_condition.is_settled(drive_error, drive_power) {
                                state.settled = true;
                            }

                            normalize_motor_power((
                                drive_power + turn_power,
                                drive_power - turn_power,
                            ), 12000.0)
                        },

                        DrivetrainTarget::DistanceAndHeading(target_distance, target_heading) => {
                            state.drive_error = target_distance - forward_travel;
                            state.turn_error = normalize_angle(heading - target_heading);

                            let drive_power = drive_controller.lock().update(state.drive_error, SAMPLE_RATE);
                            let turn_power = turn_controller.lock().update(state.turn_error, SAMPLE_RATE);

                            let (drive_error, turn_error) = (state.drive_error, state.turn_error);
                            if state.drive_settle_condition.is_settled(drive_error, drive_power) && state.turn_settle_condition.is_settled(turn_error, turn_power) {
                                state.settled = true;
                            }

                            normalize_motor_power((
                                drive_power + turn_power,
                                drive_power - turn_power,
                            ), 12000.0)
                        },

                        DrivetrainTarget::MotorPower(left_power, right_power) => {
                            state.drive_error = 0.0;
                            state.turn_error = 0.0;
                            state.settled = true;

                            (left_power, right_power)
                        },
                    };
    
                    // Set the motor voltages
                    for motor in motors.0.lock().iter_mut() {
                        motor.move_voltage((12000.0 * left_power / 100.0).round() as i32).unwrap_or(());
                    }
                    for motor in motors.1.lock().iter_mut() {
                        motor.move_voltage((12000.0 * right_power / 100.0).round() as i32).unwrap_or(());
                    }

                    // Release state mutex before delaying to allow for other locks
                    drop(state);

                    task_loop.delay();
                }
            }).unwrap()),
        }
    }

    /// Helper for setting asynchronous drivetrain targets.
    /// Essentially:
    /// - Resets drivetrain settle condition timers, since they now have a new target to get to.
    /// - Obtains a lock on the drivetrain state and sets it to a new value.
    fn set_target(&mut self, target: DrivetrainTarget) {
        let mut state = self.state.lock();

        // Reset settled state
        state.drive_settle_condition.reset();
        state.turn_settle_condition.reset();
        state.settled = false;
        
        // Set new target
        state.target = target;
    }

    /// Moves the drivetrain in a straight line for a certain distance.
    pub fn drive_distance(&mut self, distance: f64) {
        let target = self.state.lock().target;
        let (forward_travel, heading) = {
            let tracking = self.tracking.lock();
            (tracking.forward_travel(), tracking.heading())
        };

        // Add `distance` to the drivetrain's target distance.
        self.set_target(DrivetrainTarget::DistanceAndHeading(
            forward_travel + distance,
            match target {
                DrivetrainTarget::DistanceAndHeading(_, heading) => heading,
                _ => heading,
            },
        ));
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub fn turn_to_angle(&mut self, angle: f64) {
        let target = self.state.lock().target;
        let forward_travel = self.tracking.lock().forward_travel();

        // Set target heading, keeping the current forward position.
        self.set_target(DrivetrainTarget::DistanceAndHeading(
            match target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                _ => forward_travel,
            },
            angle,
        ));
    }

    /// Turns the drivetrain in place to face the direction of a certain point.
    pub fn turn_to_point(&mut self, point: Vec2) {
        let target = self.state.lock().target;
        let (position, forward_travel) = {
            let tracking = self.tracking.lock();
            (tracking.position(), tracking.forward_travel())
        };

        let displacement = point - position;
        
        // Set target heading, keeping the current forward position.
        self.set_target(DrivetrainTarget::DistanceAndHeading(
            match target {
                DrivetrainTarget::DistanceAndHeading(distance, _) => distance,
                _ => forward_travel
            },
            displacement.angle(),
        ));
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub fn move_to_point(&mut self, point: impl Into<Vec2>) {
        self.set_target(DrivetrainTarget::Point(point.into()));
    }

    /// Moves the drivetrain along a path defined by a series of waypoints.
    pub fn follow_path(&mut self, mut path: Vec<Vec2>) {
        let lookahead_distance = self.state.lock().lookahead_distance;

        let mut path_loop = Loop::new(Duration::from_millis(10));
    
        // Ensure that there is an initial intersection by inserting the current position at the start of the path.
        // This effectively creates a big starting line segment between the robot's current location and the first waypoint,
        // meaning the robot will target the first waypoint in the path even if the lookahead circle is far from it.
        path.insert(0, self.tracking.lock().position());

        // Loop through each waypoint.
        for (i, waypoint) in path.iter().enumerate() {
            // Find the next waypoint after the current one. This will form our line segment.
            // If there isn't a next waypoint, then we are targeting the final point and can stop.
            let next_waypoint = if let Some(waypoint) = path.get(i + 1) { waypoint } else { continue };

            // Move to the intersection point between the lookahead circle and the line segment formed between
            // the current and next waypoint until the lookahead circle encompasses the next waypoint.
            while self.tracking.lock().position().distance(*next_waypoint) > lookahead_distance {

                // Find intersections using the line segment + circle intersection formula.
                let intersections = LineCircleIntersections::compute_bounded(
                    (*waypoint, *next_waypoint), // Line segment from waypoint to next_waypoint
                    (self.tracking.lock().position(), lookahead_distance) // Lookahead circle
                );

                self.set_target(DrivetrainTarget::Point(match intersections {
                    // The line segment has secant intersections with the lookahead circle, resulting in two points of intersection.
                    // Choose the one cloeset to the next waypoint to ensure we go forward and not backwards on the path.
                    LineCircleIntersections::Secant(point_1, point_2) => if point_1.distance(*next_waypoint) < point_2.distance(*next_waypoint) {
                        point_1  
                    } else {
                        point_2
                    },

                    // There is one intersection (a tangent line). Move to it.
                    LineCircleIntersections::Tangent(point) => point,

                    // The lookahead circle does not intersect the waypoint line segment at all.
                    // Presumably, the robot has been greatly knocked off course, so we'll keep the last known intersection
                    // as our target to allow the drivetrain to get back on the path.
                    LineCircleIntersections::None => continue,
                }));

                path_loop.delay();
            }
        }

        self.wait_until_settled();
    }

    // Holds the current angle and position of the drivetrain.
    pub fn hold_position(&mut self) {
        let (forward_travel, heading) = {
            let tracking = self.tracking.lock();
            (tracking.forward_travel(), tracking.heading())
        };

        self.set_target(DrivetrainTarget::DistanceAndHeading(forward_travel, heading));
    }

    pub fn control_tank(&mut self, left_power: f64, right_power: f64) {
        self.set_target(DrivetrainTarget::MotorPower(left_power, right_power));
    }
    
    pub fn control_arcade(&mut self, drive_power: f64, turn_power: f64) {
        self.set_target(DrivetrainTarget::MotorPower(
            drive_power + turn_power,
            drive_power - turn_power
        ));
    }

    pub fn wait_until_settled(&self) {
        let mut spinlock = Loop::new(Duration::from_millis(10));

        while !self.state.lock().settled {
            spinlock.delay();
        }
    }

    pub fn wait_until(&self, condition: bool) {
        let mut spinlock = Loop::new(Duration::from_millis(10));

        while !condition {
            spinlock.delay();
        }
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

    pub fn lookahead_distance(&self) -> f64 {
        self.state.lock().lookahead_distance
    }

    pub fn is_settled(&self) -> bool {
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