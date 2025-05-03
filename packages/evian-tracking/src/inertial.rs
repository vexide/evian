//! Provides proper multiple inertial support

use alloc::vec::Vec;
use vexide::{
    devices::{math::Vector3, smart::imu::InertialError},
    prelude::InertialSensor,
};
/// Enum that allows you to either have a single inertial or average between several inertials
pub enum TrackingInertial {
    /// Singular inertial, will only take one inertial's angle and assume its correct
    SingleInertial(InertialSensor),
    /// More accurate multiple inertial, reduces noise by averaging several inertials
    MultipleInertial(Vec<InertialSensor>),
}

impl TrackingInertial {
    /// Returns the Inertial Sensor’s raw gyroscope readings in dps (degrees per second).
    ///
    /// # Errors
    ///
    /// - An [`InertialError::Port`] error is returned if there is not an inertial sensor connected to the port.
    /// - An [`InertialError::BadStatus`] error is returned if the inertial sensor failed to report its status.
    /// - An [`InertialError::StillCalibrating`] error is returned if the sensor is currently calibrating and cannot yet be used.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    /// use core::time::Duration;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = InertialSensor::new(peripherals.port_1);
    ///
    ///     // Calibrate sensor, panic if calibration fails.
    ///     sensor.calibrate().await.unwrap();
    ///
    ///     // Read out angular velocity values every 10mS
    ///     loop {
    ///         if let Ok(rates) = sensor.gyro_rate() {
    ///             println!(
    ///                 "x: {}°/s, y: {}°/s, z: {}°/s",
    ///                 rates.x,
    ///                 rates.y,
    ///                 rates.z,
    ///             );
    ///         }
    ///
    ///         sleep(Duration::from_millis(10)).await;
    ///     }
    /// }
    /// ```
    pub fn gyro_rate(&self) -> Result<Vector3<f64>, InertialError> {
        match self {
            Self::SingleInertial(imu) => imu.gyro_rate(),
            Self::MultipleInertial(imus) => {
                let mut avg_gyro_rate = Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                for imu in imus {
                    let rate: Result<Vector3<f64>, InertialError> = imu.gyro_rate();
                    if rate.as_ref().is_ok() {
                        let vector = rate.unwrap();
                        avg_gyro_rate.x += vector.x;
                        avg_gyro_rate.y += vector.y;
                        avg_gyro_rate.z += vector.z;
                    } else {
                        return Err(rate.err().unwrap());
                    }
                }
                avg_gyro_rate.x /= imus.len() as f64;
                avg_gyro_rate.y /= imus.len() as f64;
                avg_gyro_rate.z /= imus.len() as f64;
                Ok(avg_gyro_rate)
            }
        }
    }
    /// Returns the Inertial Sensor’s yaw angle bounded from [0.0, 360.0) degrees.
    ///
    /// Clockwise rotations are represented with positive degree values, while counterclockwise rotations are
    /// represented with negative ones.
    ///
    /// # Errors
    ///
    /// - An [`InertialError::Port`] error is returned if there is not an inertial sensor connected to the port.
    /// - An [`InertialError::BadStatus`] error is returned if the inertial sensor failed to report its status.
    /// - An [`InertialError::StillCalibrating`] error is returned if the sensor is currently calibrating and cannot yet be used.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    /// use core::time::Duration;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = InertialSensor::new(peripherals.port_1);
    ///
    ///     // Calibrate sensor, panic if calibration fails.
    ///     sensor.calibrate().await.unwrap();
    ///
    ///     // Sleep for two seconds to allow the robot to be moved.
    ///     sleep(Duration::from_secs(2)).await;
    ///
    ///     if let Ok(heading) = sensor.heading() {
    ///         println!("Heading is {} degrees.", rotation);
    ///     }
    /// }
    /// ```
    pub fn heading(&self) -> Result<f64, InertialError> {
        match self {
            Self::SingleInertial(imu) => imu.heading(),
            Self::MultipleInertial(imus) => {
                let mut avg_heading: f64 = 0.0;

                for imu in imus {
                    let heading: Result<f64, InertialError> = imu.heading();
                    if heading.as_ref().is_ok() {
                        avg_heading += heading.unwrap();
                    } else {
                        return Err(heading.err().unwrap());
                    }
                }
                avg_heading /= imus.len() as f64;
                Ok(avg_heading)
            }
        }
    }
    /// Returns the sensor's raw acceleration readings in g (multiples of ~9.8 m/s/s).
    ///
    /// # Errors
    ///
    /// - An [`InertialError::Port`] error is returned if there is not an inertial sensor connected to the port.
    /// - An [`InertialError::BadStatus`] error is returned if the inertial sensor failed to report its status.
    /// - An [`InertialError::StillCalibrating`] error is returned if the sensor is currently calibrating and cannot yet be used.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    /// use core::time::Duration;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = InertialSensor::new(peripherals.port_1);
    ///
    ///     // Calibrate sensor, panic if calibration fails.
    ///     sensor.calibrate().await.unwrap();
    ///
    ///     // Read out accleration values every 10mS
    ///     loop {
    ///         if let Ok(acceleration) = sensor.acceleration() {
    ///             println!(
    ///                 "x: {}G, y: {}G, z: {}G",
    ///                 acceleration.x,
    ///                 acceleration.y,
    ///                 acceleration.z,
    ///             );
    ///         }
    ///
    ///         sleep(Duration::from_millis(10)).await;
    ///     }
    /// }
    /// ```
    pub fn acceleration(&self) -> Result<Vector3<f64>, InertialError> {
        match self {
            Self::SingleInertial(imu) => imu.acceleration(),
            Self::MultipleInertial(imus) => {
                let mut avg_accel = Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                };
                for imu in imus {
                    let accel: Result<Vector3<f64>, InertialError> = imu.acceleration();
                    if accel.as_ref().is_ok() {
                        let vector = accel.unwrap();
                        avg_accel.x += vector.x;
                        avg_accel.y += vector.y;
                        avg_accel.z += vector.z;
                    } else {
                        return Err(accel.err().unwrap());
                    }
                }
                avg_accel.x /= imus.len() as f64;
                avg_accel.y /= imus.len() as f64;
                avg_accel.z /= imus.len() as f64;
                Ok(avg_accel)
            }
        }
    }
    /// Resets the current reading of the sensor's heading to zero.
    ///
    /// This only affects the value returned by [`InertialSensor::heading`] and does not effect [`InertialSensor::rotation`]
    /// or [`InertialSensor::euler`]/[`InertialSensor::quaternion`].
    ///
    /// # Errors
    ///
    /// - An [`InertialError::Port`] error is returned if there is not an inertial sensor connected to the port.
    /// - An [`InertialError::BadStatus`] error is returned if the inertial sensor failed to report its status.
    /// - An [`InertialError::StillCalibrating`] error is returned if the sensor is currently calibrating and cannot yet be used.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    /// use core::time::Duration;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = InertialSensor::new(peripherals.port_1);
    ///
    ///     // Calibrate sensor, panic if calibration fails.
    ///     sensor.calibrate().await.unwrap();
    ///
    ///     // Sleep for two seconds to allow the robot to be moved.
    ///     sleep(Duration::from_secs(2)).await;
    ///
    ///     // Store heading before reset.
    ///     let heading = sensor.heading().unwrap_or_default();
    ///
    ///     // Reset heading back to zero.
    ///     _ = sensor.reset_heading();
    /// }
    /// ```
    pub fn reset_heading(&mut self) -> Result<(), InertialError> {
        match self {
            Self::SingleInertial(imu) => imu.reset_heading(),
            Self::MultipleInertial(imus) => {
                for imu in imus {
                    let result: Result<(), InertialError> = imu.reset_heading();

                    result?;
                }
                Ok(())
            }
        }
    }
}
