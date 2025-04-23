//! Adaptive pure pursuit controller.

extern crate alloc;

use core::time::Duration;

use alloc::vec::Vec;
use evian_drivetrain::{Drivetrain, differential::Differential};
use evian_math::Vec2;
use evian_tracking::{TracksHeading, TracksPosition};

mod follow;
pub use follow::PurePursuitFuture;

/// Parses a [LemLib 0.5 path] into a discrete list of [`Waypoint`]s.
///
/// [LemLib 0.5 path]: https://docs.path.jerryio.com/docs/formats/LemLibFormatV0_5
pub fn parse_lemlib_path(buf: &[u8]) -> Vec<Waypoint> {
    let mut waypoints = Vec::new();

    let strbuf = str::from_utf8(buf).expect("encountered invalid UTF-8 in LemLib path");

    let data = &strbuf[0..strbuf
        .find("endData")
        .expect("missing endData delimeter in Lemlib path")];

    for line in data.lines() {
        let mut split = line.split(',');

        waypoints.push(Waypoint {
            position: Vec2 {
                x: split
                    .next()
                    .expect("missing x field in LemLib waypoint entry")
                    .trim()
                    .parse::<f64>()
                    .expect("failed to parse LemLib waypoint field `x`"),
                y: split
                    .next()
                    .expect("missing y field in LemLib waypoint entry")
                    .trim()
                    .parse::<f64>()
                    .expect("failed to parse LemLib waypoint field `y`"),
            },
            velocity: split
                .next()
                .expect("missing x field of LemLib waypoint entry")
                .trim()
                .parse::<f64>()
                .expect("failed to parse LemLib waypoint field `speed`"),
        });
    }

    waypoints
}

/// Pure pursuit path waypoint.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Position of the robot at this point along the path.
    pub position: Vec2<f64>,

    /// Linear velocity of the robot at this point along the path.
    pub velocity: f64,
}

/// Adaptive pure pursuit controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuit {
    /// Radius of the robot's lookahead circle.
    pub lookahead_distance: f64,

    /// Distance between the left and right wheels on the robot.
    pub track_width: f64,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl PurePursuit {
    /// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
    pub fn follow<'a, I: Iterator<Item = Waypoint> + Unpin, T: TracksPosition + TracksHeading>(
        &self,
        drivetrain: &'a mut Drivetrain<Differential, T>,
        waypoints: impl IntoIterator<Item = Waypoint, IntoIter = I>,
    ) -> PurePursuitFuture<'a, T, I> {
        PurePursuitFuture {
            drivetrain,
            state: None,
            waypoints: waypoints.into_iter(),
            lookahead_distance: self.lookahead_distance,
            track_width: self.track_width,
            timeout: self.timeout,
        }
    }
}
