use core::cell::RefCell;

use alloc::rc::Rc;
use evian_math::Vec2;
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

use super::{Arcade, Holonomic, DrivetrainModel};

pub struct Mecanum {
    pub front_left_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub front_right_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_left_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_right_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl DrivetrainModel for Mecanum {
    type Error = MotorError;
}

// TODO: impl Holonomic/Tank