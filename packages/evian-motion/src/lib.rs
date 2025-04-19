//! Motion control algorithms.

#![no_std]

mod basic;
pub mod pursuit;
mod seeking;

pub use basic::Basic;
pub use pursuit::PurePursuit;
pub use seeking::Seeking;
