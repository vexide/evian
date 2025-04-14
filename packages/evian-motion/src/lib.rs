//! Motion control algorithms.

#![no_std]

mod basic;
mod seeking;

pub use basic::Basic;
pub use seeking::Seeking;
