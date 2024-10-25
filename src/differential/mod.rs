//! Differential Drivetrain Control
//!
//! A differential drivetrain (also called a **tank drive**) is a robot whose movement is
//! controlled by two independently driven sets of motors on the left and right sides of its
//! chassis. The system operates by adjusting the speed and direction of the left and right
//! motors, enabling the robot to drive straight or execute turns.
//!
//! - If both sets of motors move at the same speed, the robot moves straight.
//! - If one set of motors moves faster than the other, the robot will turn.
//! - If the motors on one side move forward while the other side moves backward, the robot will rotate in place.
//!
//! Differential drivetrains are *nonholonomic*, meaning they cannot strafe laterally.

use vexide::core::float::Float;

pub mod commands;

use crate::{
    command::{Command, CommandUpdate},
    tracking::Tracking,
};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use vexide::{
    async_runtime::{
        task::{spawn, Task},
        time::sleep,
    },
    core::sync::{Barrier, Mutex},
    devices::smart::Motor,
};

/// Differential Drivetrain
///
/// # Overview
///
/// This struct combines the following systems:
///
/// - **Motor Control**: The left and right motors are represented by [`DriveMotors`], which is an alias
///   for a shared collection of motor objects. Each motor’s voltage is adjusted based on the current [`Command`].
/// - **Commands**: Commands are executed by the drivetrain, which issue motor voltages based on an
///   implementation of a motion algorithm.
/// - **Tracking**: A provided struct implementing the [`Tracking`] trait is used to gather sensor data and perform
///   localization (odometry). This data is then provided to commands, giving them updated information on the position,
///   heading, velocity, etc... of the robot at a given point in time.
pub struct DifferentialDrivetrain<T: Tracking + Send + 'static> {
    left_motors: DriveMotors,
    right_motors: DriveMotors,
    tracking: Arc<Mutex<T>>,
    command: Arc<Mutex<Option<Box<dyn Command<Output = Voltages> + Send>>>>,
    barrier: Arc<Barrier>,
    _task: Task<()>,
}

impl<T: Tracking + Send> DifferentialDrivetrain<T> {
    /// Creates a new drivetrain with the provided left/right motors and a tracking system.
    ///
    /// Motors created with the [`drive_motors`] macro may be safely cloned, as they are wrapped
    /// in an [`Arc`] to allow sharing across tasks and between the drivetrain and its tracking
    /// instance if needed.
    ///
    /// # Examples
    ///
    /// ```
    /// let left_motors = drive_motors![
    ///     Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
    ///     Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
    ///     Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
    /// ];
    /// let right_motors = drive_motors![
    ///     Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
    ///     Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
    ///     Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
    /// ];
    ///
    /// let mut drivetrain = DifferentialDrivetrain::new(
    ///     left_motors.clone(),
    ///     right_motors.clone(),
    ///     ParallelWheelTracking::new(
    ///         Vec2::default(),
    ///         90.0_f64.to_radians(),
    ///         TrackingWheel::new(left_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
    ///         TrackingWheel::new(right_motors.clone(), 3.25, 7.5, Some(36.0 / 48.0)),
    ///         None,
    ///     ),
    /// );
    /// ```
    pub fn new(left_motors: DriveMotors, right_motors: DriveMotors, tracking: T) -> Self {
        let command = Arc::new(Mutex::new(None));
        let tracking = Arc::new(Mutex::new(tracking));
        let barrier = Arc::new(Barrier::new(2));

        Self {
            left_motors: left_motors.clone(),
            right_motors: right_motors.clone(),
            tracking: tracking.clone(),
            command: command.clone(),
            barrier: barrier.clone(),
            _task: spawn(async move {
                loop {
                    let tracking_cx = tracking.lock().await.update();
                    let mut command_guard = command.lock().await;

                    if let Some(command) = command_guard.as_mut() {
                        match command.update(tracking_cx) {
                            CommandUpdate::Update(Voltages(left, right)) => {
                                for motor in left_motors.lock().await.iter_mut() {
                                    _ = motor.set_voltage(left);
                                }
                                for motor in right_motors.lock().await.iter_mut() {
                                    _ = motor.set_voltage(right);
                                }
                            }
                            CommandUpdate::Settled => {
                                *command_guard = None;
                                barrier.wait().await;
                                for motor in left_motors.lock().await.iter_mut() {
                                    _ = motor.set_voltage(0.0);
                                }
                                for motor in right_motors.lock().await.iter_mut() {
                                    _ = motor.set_voltage(0.0);
                                }
                            }
                        }
                    }

                    drop(command_guard);

                    sleep(Motor::DATA_WRITE_INTERVAL).await;
                }
            }),
        }
    }

    /// Executes a [`Command`] on the drivetrain, polling until the command has settled.
    ///
    /// The provided command implementation must have an output type of [`Voltages`] (left/right
    /// motor voltages) when updated. The future returned by this function will complete when the
    /// command has settled.
    pub async fn execute(&mut self, cmd: impl Command<Output = Voltages> + Send + 'static) {
        *self.command.lock().await = Some(Box::new(cmd));
        self.barrier.wait().await; // Await until settled.
    }

    /// Returns a shared reference to the drivetrain's [`Tracking`] system.
    ///
    /// This is returned as an `Arc<Mutex<T>>` wrapping the tracking system,
    /// and must be explicitly locked to access any sensor data off the system.
    ///
    /// # Examples
    ///
    /// ```
    /// // Obtain a lock on the drivetrain's tracking system.
    /// let lock = drivetrain.tracking().lock().await;
    ///
    /// // Read data from the system (position in this case).
    /// println!("Last known position: {:?}", lock.position());
    ///
    /// // Drop the lock so that the drivetrain's command task can use it.
    /// drop(lock);
    /// ```
    pub fn tracking(&self) -> Arc<Mutex<T>> {
        Arc::clone(&self.tracking)
    }

    /// Returns a shared reference to a possibly executing [`Command`] on the drivetrain.
    ///
    /// This is returned as an `Arc<Mutex<T>>` wrapping a dynamically-dispatched command,
    /// or `None` if no command is currently being polled by the drivetrain.
    pub fn command(&self) -> Arc<Mutex<Option<Box<dyn Command<Output = Voltages> + Send>>>> {
        Arc::clone(&self.command)
    }

    /// Returns ta shared reference to the left [`DriveMotors`].
    pub fn left_motors(&self) -> DriveMotors {
        Arc::clone(&self.left_motors)
    }

    /// Returns ta shared reference to the right [`DriveMotors`].
    pub fn right_motors(&self) -> DriveMotors {
        Arc::clone(&self.right_motors)
    }
}

// Internal alias so I don't have to type this shit out a million times.
pub type DriveMotors = Arc<Mutex<Vec<Motor>>>;

/// A macro that creates a set of motors for a [`DifferentialDrivetrain`].
///
/// This macro simplifies the creation of a [`DriveMotors`] collection, which is a sharable, threadsafe
/// wrapper around vexide's non-copyable [`Motor`](vexide::devices::smart::motor::Motor) struct.
///
/// # Examples
///
/// ```
/// let motors = drive_motors![motor1, motor2, motor3];
/// ```
#[macro_export]
macro_rules! drive_motors {
    ( $( $item:expr ),* $(,)?) => {
        {
            use ::alloc::{sync::Arc, vec::Vec};
            use ::vexide::{core::sync::Mutex, devices::smart::Motor};

            let mut temp_vec: Vec<Motor> = Vec::new();

            $(
                temp_vec.push($item);
            )*

            Arc::new(Mutex::new(temp_vec))
        }
    };
}
pub use drive_motors;

/// Left/Right Motor Voltages
///
/// Used as the standard output of a [`Command`] when working with the [`DifferentialDrivetrain`]
/// struct.
///
/// This struct is additionally a [`Command`] in itself, and can be used to run a drivetrain at a
/// fixed voltage.
///
/// [`Command`]: crate::command::Command
/// [`DifferentialDrivetrain`]: crate::differential::DifferentialDrivetrain
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Voltages(pub f64, pub f64);

impl Voltages {
    /// Returns [`Voltages`] that are less than a provided `max` value while preserving
    /// the ratio between the original left and right values.
    ///
    /// If either motor is over a `max_voltage`, both values will be decresed by the amount
    /// that is "oversaturated" to preserve the ratio between left and right power.
    pub fn normalized(&self, max: f64) -> Self {
        let larger_magnitude = self.0.abs().max(self.1.abs()) / max;

        let mut voltages = *self;

        if larger_magnitude > 1.0 {
            voltages.0 /= larger_magnitude;
            voltages.1 /= larger_magnitude;
        }

        voltages
    }

    /// Returns the left voltage.
    pub fn left(&self) -> f64 {
        self.0
    }

    /// Returns the right voltage.
    pub fn right(&self) -> f64 {
        self.1
    }
}
