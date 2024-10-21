use crate::tracking::TrackingContext;

pub trait Command {
    type Output;

    fn update(&mut self, cx: TrackingContext) -> CommandUpdate<Self::Output>;
}

pub enum CommandUpdate<T> {
    Update(T),
    Settled,
}
