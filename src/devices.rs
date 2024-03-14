use alloc::{sync::Arc, vec::Vec};
use pros::{
    core::{error::PortError, sync::Mutex},
    devices::{
        adi::{AdiEncoder, AdiError},
        smart::{
            motor::{Motor, MotorError},
            rotation::RotationSensor,
        },
        Position,
    },
};

/// Internal alias so I don't have to type this shit out a million times.
pub type DriveMotors = Arc<Mutex<Vec<Motor>>>;

/// A sensor that can measure rotation, for example, a potentiometer or encoder.
pub trait RotarySensor: Send + 'static {
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
impl_rotary_sensor!(AdiEncoder, position, AdiError);
// impl_rotary_sensor!(AdiPotentiometer, angle, AdiError); // TODO: Consider this in the future.

impl RotarySensor for Vec<Motor> {
    type Error = MotorError;

    fn position(&self) -> Result<Position, Self::Error> {
        let mut degree_sum = 0.0;

        for motor in self.iter() {
            degree_sum += motor.position()?.into_degrees();
        }

        Ok(Position::from_degrees(degree_sum / (self.len() as f64)))
    }
}

/// Blanket implementation for all Arc<Mutex<T>> wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Arc<Mutex<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        self.lock().position()
    }
}

#[macro_export]
macro_rules! drive_motors {
    ( $( $item:expr ),* $(,)?) => {
        {
            use ::alloc::{sync::Arc, vec::Vec};
            use ::pros::{core::sync::Mutex, devices::smart::Motor};

            let mut temp_vec: Vec<Motor> = Vec::new();

            $(
				if let Ok(motor) = $item {
					temp_vec.push(motor);
				}
            )*

            Arc::new(Mutex::new(temp_vec))
        }
    };
}
pub use drive_motors;
