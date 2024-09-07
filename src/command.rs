use crate::tracking::TrackingContext;

pub trait Command: Send + 'static {
    type Output;

    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output>;
}

pub enum CommandUpdate<T> {
    Update(T),
    Settled,
}
