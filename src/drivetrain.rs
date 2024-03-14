use core::{
    future::Future,
    pin::Pin,
    sync::atomic::{AtomicBool, Ordering},
    task::{Context, Poll},
};

use crate::{
    devices::DriveMotors,
    commands::Command,
    prelude::Vec2,
    tracking::{Tracking, TrackingContext},
};
use alloc::{boxed::Box, sync::Arc};
use num_traits::real::Real;
use pros::{
    devices::smart::Motor,
    core::{
        sync::Mutex,
        task::{self, TaskHandle},
    },
};

#[derive(Debug)]
pub struct DifferentialDrivetrain<T: Tracking> {
    left_motors: DriveMotors,
    right_motors: DriveMotors,
    tracking: Arc<Mutex<T>>,
    task: Option<TaskHandle>,
    started: Arc<AtomicBool>,
    command: Arc<Mutex<Box<dyn Command<Output = Voltages>>>>,
}

impl<T: Tracking> DifferentialDrivetrain<T> {
    pub fn new(left_motors: DriveMotors, right_motors: DriveMotors, tracking: T) -> Self {
        let started = Arc::new(AtomicBool::new(false));
        let command = Arc::new(Mutex::new(
            Box::new(Voltages::default())
                as Box<dyn Command<Output = Voltages>>,
        ));
        let tracking = Arc::new(Mutex::new(tracking));

        Self {
            left_motors: Arc::clone(&left_motors),
            right_motors: Arc::clone(&right_motors),
            tracking: Arc::clone(&tracking),
            command: Arc::clone(&command),
            started: Arc::clone(&started),
            task: Some(task::spawn(move || {
                while started.load(Ordering::Relaxed) {
                    let tracking_ctx = tracking.lock().update();
                    let Voltages(left, right) = command.lock().update(tracking_ctx);

                    for motor in left_motors.lock().iter_mut() {
                        motor.set_voltage(left).unwrap();
                    }
                    for motor in right_motors.lock().iter_mut() {
                        motor.set_voltage(right).unwrap();
                    }

                    task::delay(Motor::DATA_READ_RATE);
                }
            })),
        }
    }

    pub fn execute(
        &mut self,
        cmd: impl Command<Output = Voltages>,
    ) -> SettleFuture<Voltages> {
        let mut command = self.command.lock();
        *command = Box::new(cmd) as Box<dyn Command<Output = Voltages>>;

        SettleFuture(self.command.clone())
    }

    pub fn position(&self) -> Vec2   {
        self.tracking.lock().position()
    }

    pub fn heading(&self) -> f64 {
        self.tracking.lock().heading()
    }

    pub fn forward_travel(&self) -> f64 {
        self.tracking.lock().forward_travel()
    }

    pub fn is_settled(&self) -> bool {
        self.command.lock().is_settled()
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

impl<T: Tracking> Drop for DifferentialDrivetrain<T> {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            self.started.swap(false, Ordering::Relaxed);
            task.join();
        }
    }
}

pub struct SettleFuture<O: 'static>(pub(crate) Arc<Mutex<Box<dyn Command<Output = O>>>>);

impl<O: 'static> Future for SettleFuture<O> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        match self.0.lock().is_settled() {
            true => Poll::Ready(()),
            false => {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }
}

/// Left/Right Motor Voltages
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