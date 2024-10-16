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

use super::Voltages;

pub struct DifferentialDrivetrain<T: Tracking> {
    left_motors: DriveMotors,
    right_motors: DriveMotors,
    tracking: Arc<Mutex<T>>,
    command: Arc<Mutex<Option<Box<dyn Command<Output = Voltages>>>>>,
    barrier: Arc<Barrier>,
    _task: Task<()>,
}

impl<T: Tracking> DifferentialDrivetrain<T> {
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
                    let mut command_guard = command.lock().await;

                    if let Some(command) = command_guard.as_mut() {
                        match command.update(tracking.lock().await.update()) {
                            CommandUpdate::Update(Voltages(left, right)) => {
                                for motor in left_motors.lock().await.iter_mut() {
                                    motor.set_voltage(left).unwrap();
                                }
                                for motor in right_motors.lock().await.iter_mut() {
                                    motor.set_voltage(right).unwrap();
                                }
                            }
                            CommandUpdate::Settled => {
                                *command_guard = None;
                                barrier.wait();
                                for motor in left_motors.lock().await.iter_mut() {
                                    motor.set_voltage(0.0).unwrap();
                                }
                                for motor in right_motors.lock().await.iter_mut() {
                                    motor.set_voltage(0.0).unwrap();
                                }
                            }
                        }
                    }

                    drop(command_guard);

                    sleep(Motor::DATA_READ_INTERVAL).await;
                }
            }),
        }
    }

    pub async fn execute(&mut self, cmd: impl Command<Output = Voltages>) {
        *self.command.lock().await = Some(Box::new(cmd));
        self.barrier.wait().await;
    }

    pub fn tracking(&self) -> Arc<Mutex<T>> {
        Arc::clone(&self.tracking)
    }

    pub fn command(&self) -> Arc<Mutex<Option<Box<dyn Command<Output = Voltages>>>>> {
        Arc::clone(&self.command)
    }

    pub fn left_motors(&self) -> DriveMotors {
        Arc::clone(&self.left_motors)
    }

    pub fn right_motors(&self) -> DriveMotors {
        Arc::clone(&self.right_motors)
    }
}

/// Internal alias so I don't have to type this shit out a million times.
pub type DriveMotors = Arc<Mutex<Vec<Motor>>>;

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
