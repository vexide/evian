//! Motion control algorithms.

#![no_std]

// extern crate alloc;

mod basic;
mod seeking;

pub use basic::Basic;
pub use seeking::Seeking;
