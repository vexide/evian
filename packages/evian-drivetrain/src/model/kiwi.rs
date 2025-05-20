use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

use crate::DrivetrainModel;

pub struct Kiwi {
    pub front_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_left_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_right_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl DrivetrainModel for Kiwi {
    type Error = MotorError;
}

// TODO: impl Holonomic/Arcade