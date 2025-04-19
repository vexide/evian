//! Motion control algorithms.

#![no_std]

mod basic;
mod seeking;
pub mod pursuit;

pub use basic::Basic;
pub use seeking::Seeking;
pub use pursuit::PurePursuit;