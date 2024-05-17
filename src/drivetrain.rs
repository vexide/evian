use core::{
    future::Future,
    pin::{pin, Pin},
    sync::atomic::{AtomicBool, Ordering},
    task::{Context, Poll},
};

use crate::{
    commands::Command,
    devices::DriveMotors,
    tracking::{Tracking, TrackingContext},
};
use alloc::{boxed::Box, sync::Arc};
use num_traits::real::Real;
use replace_with::replace_with;
use vexide::{
    async_runtime::{
        task::{spawn, Task},
        time::sleep,
    },
    core::sync::{Mutex, MutexGuard, MutexLockFuture},
    devices::smart::Motor,
};

#[derive(Debug)]
pub struct DifferentialDrivetrain<T: Tracking> {
    left_motors: DriveMotors,
    right_motors: DriveMotors,
    tracking: Arc<Mutex<T>>,
    command: Arc<Mutex<Box<dyn Command<Output = Voltages>>>>,
    _task: Task<()>,
}

impl<T: Tracking> DifferentialDrivetrain<T> {
    pub fn new(left_motors: DriveMotors, right_motors: DriveMotors, tracking: T) -> Self {
        let started = Arc::new(AtomicBool::new(false));
        let command = Arc::new(Mutex::new(
            Box::new(Voltages::default()) as Box<dyn Command<Output = Voltages>>
        ));
        let tracking = Arc::new(Mutex::new(tracking));

        Self {
            left_motors: Arc::clone(&left_motors),
            right_motors: Arc::clone(&right_motors),
            tracking: Arc::clone(&tracking),
            command: Arc::clone(&command),
            _task: spawn(async move {
                while started.load(Ordering::Relaxed) {
                    let Voltages(left, right) =
                        command.lock().await.update(tracking.lock().await.update());

                    for motor in left_motors.lock().await.iter_mut() {
                        motor.set_voltage(left).unwrap();
                    }
                    for motor in right_motors.lock().await.iter_mut() {
                        motor.set_voltage(right).unwrap();
                    }

                    sleep(Motor::DATA_READ_INTERVAL).await;
                }
            }),
        }
    }

    pub fn execute(&mut self, cmd: impl Command<Output = Voltages>) -> Execute<Voltages> {
        Execute::SetCommand {
            guard: self.command.lock(),
            command: Box::new(cmd) as Box<dyn Command<Output = Voltages>>,
        }
    }

    pub fn tracking(&self) -> Arc<Mutex<T>> {
        Arc::clone(&self.tracking)
    }

    pub fn command(&self) -> Arc<Mutex<Box<dyn Command<Output = Voltages>>>> {
        Arc::clone(&self.command)
    }

    pub fn left_motors(&self) -> DriveMotors {
        Arc::clone(&self.left_motors)
    }

    pub fn right_motors(&self) -> DriveMotors {
        Arc::clone(&self.right_motors)
    }
}

pub enum Execute<'a, O: 'static> {
    SetCommand {
        guard: MutexLockFuture<'a, Box<dyn Command<Output = O>>>,
        command: Box<dyn Command<Output = O>>,
    },
    Settle(MutexGuard<'a, Box<dyn Command<Output = O>>>),
}

impl<'a, O: 'static> Future for Execute<'a, O> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut is_settled = false;

        replace_with(
            &mut *self,
            || panic!("Failed to replace"),
            |self_| match self_ {
                Self::SetCommand { mut guard, command } => match pin!(&mut guard).poll(cx) {
                    Poll::Ready(mut lock) => {
                        *lock = command;
                        Self::Settle(lock)
                    }
                    Poll::Pending => Self::SetCommand { guard, command },
                },
                Self::Settle(lock) => {
                    if lock.is_settled() {
                        is_settled = true;
                    }

                    Self::Settle(lock)
                }
            },
        );

        if is_settled {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

/// Left/Right Motor Voltages
/// 
/// Used as the standard output of a [`Command`] when working with the [`DifferentialDrivetrain`]
/// struct.
///
/// This struct is additionally a [`Command`] in itself, and can be used to run a drivetrain at a
/// fixed voltage.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Voltages(pub f64, pub f64);

impl Voltages {
    /// Normalizes the ratio of voltages between two motors.
    ///
    /// If either motor is over a `max_voltage`, limit both voltages to preserve
    /// the ratio between left and right power.
    pub fn normalized(&self, max: f64) -> Self {
        let larger_voltage = self.0.abs().max(self.1.abs()) / max;

        let mut voltages = self.clone();

        if larger_voltage > 1.0 {
            voltages.0 /= larger_voltage;
            voltages.1 /= larger_voltage;
        }

        return voltages;
    }
}

impl From<(f64, f64)> for Voltages {
    fn from(tuple: (f64, f64)) -> Self {
        Self(tuple.0, tuple.1)
    }
}

impl Command for Voltages {
    type Output = Self;

    fn update(&mut self, _ctx: TrackingContext) -> Self {
        self.clone()
    }

    fn is_settled(&self) -> bool {
        true
    }

    fn cancel(&mut self) {}
}