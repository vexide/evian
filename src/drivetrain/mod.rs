pub mod differential;

/// A mobile robot drivetrain capable of measuring data about itself.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Drivetrain<M, T> {
    pub motors: M,
    pub tracking: T,
}

impl<M, T> Drivetrain<M, T> {
    pub const fn new(motors: M, tracking: T) -> Self {
        Self { motors, tracking }
    }
}
