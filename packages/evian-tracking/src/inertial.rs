//! Provides a nice method of making your own inertial devices

use alloc::borrow::ToOwned;
use evian_math::Angle;
use vexide::prelude::Float;
use vexide::{devices::smart::imu::InertialError, prelude::InertialSensor};

/// Generic IMU trait, allowing for use of anything that implements this as a method of getting heading
pub trait Gyro {
    /// Implementation defined error type
    type Error;

    /// Method of retreiving heading
    fn gyro_heading(&mut self) -> Result<Angle, Self::Error>;
    /// get raw angular velocity readings
    fn gyro_angular_velocity(&mut self) -> Result<f64, Self::Error>;
}

impl Gyro for InertialSensor {
    type Error = InertialError;
    fn gyro_angular_velocity(&mut self) -> Result<f64, Self::Error> {
        let gyro_rate = self.gyro_rate();
        match gyro_rate {
            Ok(s) => Ok(s.z),
            Err(e) => Err(e),
        }
    }

    fn gyro_heading(&mut self) -> Result<Angle, Self::Error> {
        let heading = self.heading();

        match heading {
            Ok(s) => Ok(Angle::from_radians(s)),
            Err(e) => Err(e),
        }
    }
}
/// 7.5 radian degree tolerance
const MAX_TOLERANCE_RADIANS: f64 = 7.5;
const MAX_TOLERANCE_G: f64 = 1.5;
impl<const N: usize> Gyro for [InertialSensor; N] {
    type Error = [Option<InertialError>; N];
    fn gyro_angular_velocity(&mut self) -> Result<f64, Self::Error> {
        let mut success_value: [f64; N] = [const { 0.0 }; N];
        let mut errors: Self::Error = [const { None }; N];
        let mut succeeded: bool = false;

        for (index, imu) in self.iter_mut().enumerate() {
            let angular_vel = imu.gyro_rate();
            match angular_vel {
                Ok(s) => {
                    success_value[index] = s.z;
                    succeeded = true;
                }
                Err(e) => {
                    errors[index] = Some(e);
                }
            }
        }

        let mut avg: f64 = 0.0;
        // calculate initial average
        drop(success_value.iter().map(|vel| avg += vel.to_owned()));
        avg /= N as f64;
        // z-score outlier detection
        let mut variance: f64 = 0.0;
        drop(success_value.iter().map(|vel| variance += vel.to_owned()));
        variance /= N as f64 - 1.0;
        let standard_deviation = f64::sqrt(variance);
        let mut ignored: u8 = 0;
        for (index, heading) in success_value.into_iter().enumerate() {
            let z_score = (heading - avg) / standard_deviation;
            if f64::abs(z_score) >= MAX_TOLERANCE_G {
                success_value[index] = 0.0;
                ignored += 1;
            }
        }
        // calculate new average
        drop(success_value.iter().map(|vel| avg += vel.to_owned()));
        avg /= (N - ignored as usize) as f64;
        if !succeeded {
            return Err(errors);
        }
        Ok(avg)
    }
    fn gyro_heading(&mut self) -> Result<Angle, Self::Error> {
        let mut success_value: [Angle; N] = [const { Angle::from_radians(0.0) }; N];
        let mut errors: Self::Error = [const { None }; N];
        let mut succeeded: bool = false;
        for (index, imu) in self.iter_mut().enumerate() {
            let angle = imu.heading();
            match angle {
                Ok(s) => {
                    success_value[index] = Angle::from_radians(s);
                    succeeded = true;
                }
                Err(e) => {
                    errors[index] = Some(e);
                }
            }
        }

        let mut avg: Angle = Angle::from_radians(0.0);
        // calculate initial average
        drop(success_value.iter().map(|angle| avg += angle.to_owned()));
        avg /= N as f64;
        // z-score outlier detection
        let mut variance: f64 = 0.0;
        drop(
            success_value
                .iter()
                .map(|angle| variance += angle.to_owned().as_radians()),
        );
        variance /= N as f64 - 1.0;
        let standard_deviation = f64::sqrt(variance);
        let mut ignored: u8 = 0;
        for (index, heading) in success_value.into_iter().enumerate() {
            let z_score = (heading - avg) / standard_deviation;
            if f64::abs(z_score.as_radians()) >= MAX_TOLERANCE_RADIANS {
                success_value[index] = Angle::from_radians(0.0);
                ignored += 1;
            }
        }
        // calculate new average
        drop(success_value.iter().map(|angle| avg += angle.to_owned()));
        avg /= (N - ignored as usize) as f64;
        if !succeeded {
            return Err(errors);
        }
        Ok(avg)
    }
}
