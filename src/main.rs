#![no_std]
#![no_main]

extern crate alloc;

use vex_rt::prelude::*;

struct HelloBot;

impl Robot for HelloBot {
    fn new(_peripherals: Peripherals) -> Self {
        println!("Hello, world");
        HelloBot
    }
}

entry!(HelloBot);
