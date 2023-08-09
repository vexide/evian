use alloc::{sync::Arc, vec::Vec};
use core::time::Duration;
use num_traits::real::Real;

use crate::{controller::FeedbackController, math::{Vec2, normalize_angle, normalize_motor_voltages}, tracking::Tracking};
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

/// A struct internal to [`DifferentialDrivetrain`] that contains fields shared between the
/// main thread and the drivetrain's background thread.
struct ThreadedDifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    motors: (Vec<Motor>, Vec<Motor>),
    tracking: T,
    thread_active: bool,
    distance_controller: U,
    heading_controller: V,
    linear_tolerance: f64,
    angular_tolerance: f64,
    lookahead_distance: f64,
    target: DrivetrainTarget,
    settled: bool,
}

pub struct DifferentialDrivetrain<T: Tracking, U: FeedbackController, V: FeedbackController> {
    thread: Option<Task>,
    inner: Arc<Mutex<ThreadedDifferentialDrivetrain<T, U, V>>>,
}

impl<
        T: Tracking + Send + 'static,
        U: FeedbackController + Send + 'static,
        V: FeedbackController + Send + 'static,
    > DifferentialDrivetrain<T, U, V>
{
    pub fn new(
        motors: (Vec<Motor>, Vec<Motor>),
        tracking: T,
        distance_controller: U,
        heading_controller: V,
        linear_tolerance: f64,
        angular_tolerance: f64,
        lookahead_distance: f64,
    ) -> Self {
        Self {
            thread: None,
            inner: Arc::new(Mutex::new(ThreadedDifferentialDrivetrain {
                motors,
                tracking,
                distance_controller,
                heading_controller,
                lookahead_distance,
                linear_tolerance,
                angular_tolerance,
                thread_active: false,
                target: DrivetrainTarget::DistanceAndHeading(0.0, 0.0),
                settled: false,
            })),
        }
    }

    /// Spawns a thread that continiously updates the drivetrain's internal [`Tracking`] instance
    /// in a 10-millisecond interval. This thread will run in the background until [`DifferentialDrivetrain::stop_tracking`]
    /// is called.
    pub fn enable(&mut self) {
        let inner = Arc::clone(&self.inner);

        self.thread = Some(
            Task::spawn(move || {
                let mut inner = inner.lock();
                let mut task_loop = Loop::new(Duration::from_millis(10));

                while inner.thread_active {
                    inner.tracking.update();

                    let (distance_error, heading_error) = match inner.target {
                        DrivetrainTarget::Point(point) => {
                            let local_target = point - inner.tracking.position();
                            
                            let heading_error = normalize_angle(inner.tracking.heading() - local_target.angle());
                            let distance_error = local_target.length() * heading_error.cos();

                            (distance_error, heading_error)
                        },
                        DrivetrainTarget::DistanceAndHeading(distance, heading) => (
                            distance - inner.tracking.forward_travel(),
                            normalize_angle(heading - inner.tracking.heading())
                        )
                    };

                    let distance_output = inner.distance_controller.update(distance_error);
                    let heading_output = inner.heading_controller.update(heading_error);

                    let (left_voltage, right_voltage) = normalize_motor_voltages((
                        distance_output + heading_output,
                        distance_output - heading_output
                    ), 128.0);
                    
                    for motor in inner.motors.0.iter_mut() {
                        motor.move_voltage(left_voltage.round() as i32).unwrap();
                    }
                    for motor in inner.motors.1.iter_mut() {
                        motor.move_voltage(right_voltage.round() as i32).unwrap();
                    }

                    task_loop.delay();
                }

                // Stop motors once done.
                for motor in inner.motors.0.iter_mut() { motor.move_voltage(0).unwrap(); }
                for motor in inner.motors.1.iter_mut() { motor.move_voltage(0).unwrap(); }
            })
            .unwrap(),
        );
    }

    pub fn disable(&self) {
        if let Some(thread) = &self.thread {
            let inner = Arc::clone(&self.inner);
            let mut inner = inner.lock();
            inner.thread_active = false;
            drop(inner);

            unsafe {
                thread.delete();
            }
        }
    }

    // pub fn tracking(&self) -> &impl Tracking {
    //     &self.tracking
    // }

    /// Moves the drivetrain in a straight line for a certain distance, then stops and holds
    /// the final position once in tolerance.
    pub async fn drive_distance(&mut self, distance: f64) {
        if self.thread.is_some() {
            let inner = Arc::clone(&self.inner);
            let mut inner = inner.lock();

            inner.target = DrivetrainTarget::DistanceAndHeading(
                inner.tracking.forward_travel() + distance,
                inner.tracking.heading()
            );
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }

    /// Turns the drivetrain in place to face a certain angle.
    pub async fn turn_to_angle(&self, angle: f64) {
        if self.thread.is_some() {
            let inner = Arc::clone(&self.inner);
            let mut inner = inner.lock();

            inner.target = DrivetrainTarget::DistanceAndHeading(
                inner.tracking.forward_travel(),
                angle
            );
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }
    
    /// Turns the drivetrain in place to face a certain point.
    pub async fn turn_to_point(&self, _point: Vec2) {
        if self.thread.is_some() {
            todo!();
            // let inner = Arc::clone(&self.inner);
            // let mut inner = inner.lock();

            // inner.target = DrivetrainTarget::DistanceAndHeading(
            //     inner.tracking.forward_travel(),
            //     angle
            // );
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }

    /// Moves the drivetrain to a certain point by turning and driving at the same time.
    pub async fn move_to_point(&self, point: Vec2) {
        if self.thread.is_some() {
            let inner = Arc::clone(&self.inner);
            let mut inner = inner.lock();

            inner.target = DrivetrainTarget::Point(point);
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }

    /// Moves the drivertain along a path defined by a series of waypoints.
    pub async fn follow_path(&self, _path: Vec<Vec2>) {
        if self.thread.is_some() {
            todo!();
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }

    // Holds the current angle and position of the drivetrain.
    pub async fn hold_position(&self) {
        if self.thread.is_some() {
            let inner = Arc::clone(&self.inner);
            let mut inner = inner.lock();

            inner.target = DrivetrainTarget::DistanceAndHeading(
                inner.tracking.forward_travel(),
                inner.tracking.heading()
            );
        } else {
            panic!("Drivetrain task is not running. Consider running start_task() beforehand.");
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;

    struct FakeSensor {}

    impl RotarySensor for FakeSensor {
        fn rotation(&self) -> f64 {
            0.0
        }
        fn reset_rotation(&self) {}
    }

    #[test]
    fn test() {
        let drivetrain = DifferentialDrivetrain::new(
            (vec![front_left, back_left], vec![front_right, back_right]),
            ParallelWheelTracking::new(
                Vec2::new(0.0, 0.0),
                90.0,
                TrackingWheel::new(vec![front_left, back_left], 3.25, Some(84.0 / 60.0)),
                TrackingWheel::new(vec![front_right, back_right], 3.25, Some(84.0 / 60.0)),
                HeadingMethod::TrackWidth(12.5),
            ),
            PIDController::new(0.0, 0.0, 0.0),
            PIDController::new(0.0, 0.0, 0.0),
            0.5,
            0.5,
            0.5,
        );
    }
}

// impl DifferentialDrivetrain {

// };
