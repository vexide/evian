pub mod differential;

/// A mobile robot capable of measuring data about itself.
#[derive(Default, Debug, Eq, PartialEq, Hash)]
pub struct Drivetrain<D, T> {
    pub motors: D,
    pub tracking: T,
}

impl<D, T> Drivetrain<D, T> {
    pub const fn new(motors: D, tracking: T) -> Self {
        Self { motors, tracking }
    }
}
