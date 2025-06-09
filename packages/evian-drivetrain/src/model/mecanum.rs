use core::cell::RefCell;

use alloc::rc::Rc;
use evian_math::{Vec2, desaturate};
use vexide::{devices::smart::motor::MotorError, prelude::Motor};

use crate::model::Tank;

use super::{Arcade, DrivetrainModel, Holonomic};

pub struct Mecanum {
    pub front_left_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub front_right_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_left_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub back_right_motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

impl DrivetrainModel for Mecanum {
    type Error = MotorError;
}

impl Holonomic for Mecanum {
    fn drive_vector(&mut self, vector: Vec2<f64>, turn: f64) -> Result<(), Self::Error> {
        let [fl, fr, bl, br] = desaturate(
            [
                vector.y + vector.x + turn,
                vector.y - vector.x - turn,
                vector.y - vector.x + turn,
                vector.y + vector.x - turn,
            ],
            1.0,
        );

        let mut rtn = Ok(());

        for motor in self.front_left_motors.borrow_mut().as_mut() {
            let result = motor.set_voltage(fl * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }
        for motor in self.front_right_motors.borrow_mut().as_mut() {
            let result = motor.set_voltage(fr * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }
        for motor in self.back_left_motors.borrow_mut().as_mut() {
            let result = motor.set_voltage(bl * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }
        for motor in self.back_right_motors.borrow_mut().as_mut() {
            let result = motor.set_voltage(br * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}

impl Tank for Mecanum {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        let mut rtn = Ok(());

        for motor in self
            .front_left_motors
            .borrow_mut()
            .as_mut()
            .iter_mut()
            .chain(self.back_left_motors.borrow_mut().as_mut())
        {
            let result = motor.set_voltage(left * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }
        for motor in self
            .front_right_motors
            .borrow_mut()
            .as_mut()
            .iter_mut()
            .chain(self.back_right_motors.borrow_mut().as_mut())
        {
            let result = motor.set_voltage(right * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}
