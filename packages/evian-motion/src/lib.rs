//! Motion control algorithms.

#![no_std]

mod curvature;

pub mod basic;
pub mod pursuit;
pub mod seeking;

pub use basic::Basic;
pub use curvature::CurvatureDrive;
pub use pursuit::PurePursuit;
pub use seeking::Seeking;
