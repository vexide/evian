use alloc::{sync::Arc, vec::Vec};
use vexide::{
    core::sync::{Mutex, MutexGuard},
    devices::{
        adi::{encoder::EncoderError, AdiEncoder},
        position::Position,
        smart::{
            motor::{Motor, MotorError},
            rotation::RotationSensor,
        },
        PortError,
    },
};

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor {
    type Error;

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
impl_rotary_sensor!(AdiEncoder, position, EncoderError);

impl RotarySensor for Vec<Motor> {
    type Error = MotorError;

    fn position(&self) -> Result<Position, Self::Error> {
        let mut degree_sum = 0.0;

        for motor in self.iter() {
            degree_sum += motor.position()?.as_degrees();
        }

        Ok(Position::from_degrees(degree_sum / (self.len() as f64)))
    }
}

/// Blanket implementation for all Arc<Mutex<T>> wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Arc<Mutex<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        let mut guard: Option<MutexGuard<'_, T>> = None;

        while match self.try_lock() {
            Some(lock) => {
                guard = Some(lock);
                false
            }
            None => true,
        } {
            core::hint::spin_loop();
        }

        guard.unwrap().position()
    }
}
