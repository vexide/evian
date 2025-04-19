//! Motion control algorithms.

#![no_std]

pub mod basic;
pub mod pursuit;
pub mod seeking;

pub use basic::Basic;
pub use pursuit::PurePursuit;
pub use seeking::Seeking;
