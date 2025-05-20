use core::cell::RefCell;

use alloc::{rc::Rc, vec::Vec};
use evian_math::Angle;
use vexide::devices::smart::imu::{InertialError, InertialSensor};
use vexide::devices::{
    PortError,
    adi::AdiEncoder,
    position::Position,
    smart::{
        motor::{Motor, MotorError},
        rotation::RotationSensor,
    },
};

/// A sensor that can measure continuous angular rotation, such as an encoder.
pub trait RotarySensor {
    /// The type of error that the device returns when [`RotarySensor::position`]
    /// fails to return a value.
    type Error;

    /// Reads the angular position of the sensor.
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

impl<T: RotarySensor> RotarySensor for Vec<T> {
    type Error = T::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        // The total motors to be used in the average later
        let mut total_motors = 0;
        let mut degree_sum = 0.0;
        let mut last_error = None;

        for motor in self {
            degree_sum += match motor.position() {
                Ok(position) => {
                    total_motors += 1;
                    position.as_degrees()
                }
                Err(error) => {
                    // Since this motor isn't being counted in the average, decrement the count
                    last_error = Some(error);
                    continue;
                }
            };
        }

        // Handle a case where no motors were added to the total.
        if total_motors == 0 {
            return if let Some(error) = last_error {
                // Return the error from the last motor that failed.
                Err(error)
            } else {
                // This means there were no motors in the group. We don't want to divide by zero here.
                Ok(Position::default())
            };
        }

        #[allow(clippy::cast_precision_loss)]
        Ok(Position::from_degrees(degree_sum / f64::from(total_motors)))
    }
}

// Duplicated because rust is stupid and I can't blanket-impl this for AsMut<[T]>.
impl<const N: usize, T: RotarySensor> RotarySensor for [T; N] {
    type Error = T::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        // The total motors to be used in the average later
        let mut total_motors = 0;
        let mut degree_sum = 0.0;
        let mut last_error = None;

        for motor in self {
            degree_sum += match motor.position() {
                Ok(position) => {
                    total_motors += 1;
                    position.as_degrees()
                }
                Err(error) => {
                    // Since this motor isn't being counted in the average, decrement the count
                    last_error = Some(error);
                    continue;
                }
            };
        }

        // Handle a case where no motors were added to the total.
        if total_motors == 0 {
            return if let Some(error) = last_error {
                // Return the error from the last motor that failed.
                Err(error)
            } else {
                // This means there were no motors in the group. We don't want to divide by zero here.
                Ok(Position::default())
            };
        }

        #[allow(clippy::cast_precision_loss)]
        Ok(Position::from_degrees(degree_sum / f64::from(total_motors)))
    }
}

/// Blanket implementation for all `Rc<RefCell<T>>` wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Rc<RefCell<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Position, Self::Error> {
        self.borrow().position()
    }
}

/// A "gyroscope," or a sensor that measures the robot's heading and angular velocity
pub trait Gyro {
    /// The error returned when [`Gyro::heading`] or [`Gyro::angular_velocity`] fails. This
    /// describes a hardware error.
    type Error;

    /// Returns the heading of the robot as an [`Angle`]
    fn heading(&self) -> Result<Angle, Self::Error>;
    /// Returns the horizontal angular velocity
    fn angular_velocity(&self) -> Result<f64, Self::Error>;
}

impl Gyro for InertialSensor {
    type Error = InertialError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        InertialSensor::heading(self).map(Angle::from_radians)
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        InertialSensor::gyro_rate(self).map(|rate| rate.z)
    }
}
impl<const N: usize> Gyro for [InertialSensor; N] {
    type Error = [Option<InertialError>; N];

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        Ok(1.0)
    }
    fn heading(&self) -> Result<Angle, Self::Error> {
        Ok(Angle::from_degrees(1.0))
    }
}
