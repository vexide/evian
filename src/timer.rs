use core::time::Duration;
use vex_rt::rtos::{Instant, time_since_start};

pub trait Timer {
	fn now() -> Self;
	fn elapsed(&self) -> Duration;
}

impl Timer for Instant {
	fn now() -> Self {
		time_since_start()
	}

	fn elapsed(&self) -> Duration {
		Duration::from_micros(time_since_start().as_micros() - self.as_micros())
	}
}