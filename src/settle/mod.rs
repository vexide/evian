use num_traits::real::Real;

use crate::tracking::TrackingContext;

pub trait SettleCondition {
	fn is_settled(&mut self, ctx: TrackingContext) -> bool;
}

pub struct JoinedSettlers {
}

pub struct Tolerance {
	value: f64,
	tolerance: f64,
}

impl SettleCondition for Tolerance {
	fn is_settled(&mut self) -> bool {
		self.value.abs() < self.tolerance
	}
}