use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

use crate::DrivetrainModel;

pub struct HDrive {
    pub left: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub right: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub sideways: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl DrivetrainModel for HDrive {
    type Error = MotorError;
}

// TODO: impl Holonomic/Tank
