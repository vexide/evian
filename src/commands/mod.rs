use crate::tracking::TrackingContext;

pub mod basic;
pub mod joystick;

pub trait Command: core::fmt::Debug + Send + 'static {
    type Output;

    fn update(&mut self, ctx: TrackingContext) -> Self::Output;
    fn cancel(&mut self);
    fn is_settled(&self) -> bool;
}
