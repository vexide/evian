use core::cell::RefCell;

use alloc::{rc::Rc, vec::Vec};
use vexide::devices::{
    adi::AdiEncoder,
    position::Position,
    smart::{
        motor::{Motor, MotorError},
        rotation::RotationSensor,
    },
    PortError,
};

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor {
    type Error;

    /// Reads the angular position measurement of the sensor.
    ///
    /// # Errors
    ///
    /// Returns [`Self::Error`] if the reading failed.
    fn position(&self) -> Result<Position, Self::Error>;
}

macro_rules! impl_rotary_sensor {
    ( $struct:ident, $method:ident, $err:ty) => {
        impl RotarySensor for $struct {
            type Error = $err;

            fn position(&self) -> Result<Position, Self::Error> {
                $struct::$method(&self)
            }
        }
    };
}

impl_rotary_sensor!(Motor, position, MotorError);
impl_rotary_sensor!(RotationSensor, position, PortError);
impl_rotary_sensor!(AdiEncoder, position, PortError);

impl RotarySensor for Vec<Motor> {
    type Error = MotorError;

    fn position(&self) -> Result<Position, Self::Error> {
        let mut degree_sum = 0.0;
        // The total motors to be used in the average later
        let mut total_motors = self.len();

        for motor in self {
            degree_sum += if let Ok(position) = motor.position() {
                position.as_degrees()
            } else {
                // Since this motor isn't being counted in the average, decrement the count
                total_motors -= 1;
                continue;
            };
        }

        #[allow(clippy::cast_precision_loss)]
        Ok(Position::from_degrees(degree_sum / (total_motors as f64)))
    }
}

/// Blanket implementation for all `Rc<RefCell<T>>` wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Rc<RefCell<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        self.borrow().position()
    }
}
