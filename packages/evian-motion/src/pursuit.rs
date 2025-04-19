//! Adaptive pure pursuit.

extern crate alloc;

use core::time::Duration;

use alloc::vec::Vec;
use evian_drivetrain::{
    Drivetrain,
    differential::{Differential, Voltages},
};
use evian_math::{Angle, Vec2};
use evian_tracking::{TracksHeading, TracksPosition};
use vexide::{
    float::Float,
    prelude::Motor,
    time::{Instant, sleep},
};

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
    /// Follows a given [`PursuitPath`].
    pub async fn follow(
        &self,
        drivetrain: &mut Drivetrain<Differential, impl TracksPosition + TracksHeading>,
        waypoints: impl IntoIterator<Item = Waypoint>,
    ) {
        let mut waypoints = waypoints.into_iter();

        let Some(mut next) = waypoints.next() else {
            return; // path is empty
        };

        // We want an initial intersection to handle the case where the robot
        // starts entirely off the path, so add a "fake waypoint" to the path at
        // the robot's starting position with the first real waypoint's velocity.
        //
        // This works okay for cases where the robot starts positioned reasonably
        // close to the first waypoint in the path.
        let position = drivetrain.tracking.position();
        let mut current = Waypoint {
            position,
            velocity: next.velocity,
        };

        // Keep iterating line segments until we find one we haven't intersected.
        while position.distance(next.position) < self.lookahead_distance {
            current = next;
            next = if let Some(next_waypoint) = waypoints.next() {
                next_waypoint
            } else {
                return;
            };
        }

        // Compute initial lookahead point.
        let mut lookahead_point = match Self::line_segment_circle_intersections(
            position,
            self.lookahead_distance,
            current.position,
            next.position,
        ) {
            // No initial intersections, shouldn't be possible since we inserted the
            // current position into the start of the path.
            (None, None) => unreachable!(),

            // One intersection; use that.
            (Some(solution), None) | (None, Some(solution)) => solution,

            // Two intersections, pick whichever one is closest to the next point on the path.
            (Some(solution_1), Some(solution_2)) => {
                if solution_1.distance(next.position) < solution_2.distance(next.position) {
                    solution_1
                } else {
                    solution_2
                }
            }
        };

        let start_time = Instant::now();

        loop {
            sleep(Duration::from_millis(5)).await;

            // Cancel movement if timeout has elapsed.
            if self
                .timeout
                .is_some_and(|timeout| start_time.elapsed() > timeout)
            {
                break;
            }

            // Tracking data (robot position and heading)
            let position = drivetrain.tracking.position();
            let heading = drivetrain.tracking.heading();

            // If the lookahead circle envelops the end of the current waypoint segment,
            // then switch to the next two waypoints.
            //
            // The ending point of the current segment becomes the starting point of the
            // next and so on until the path is complete.
            while position.distance(next.position) < self.lookahead_distance {
                current = next;
                next = if let Some(next_waypoint) = waypoints.next() {
                    next_waypoint
                } else {
                    // We're out of waypoints, meaning the end of path has been reached.
                    break;
                };
            }

            // Compute lookahead point.
            match Self::line_segment_circle_intersections(
                position,
                self.lookahead_distance,
                current.position,
                next.position,
            ) {
                // No intersections; the lookahead circle isn't intersecting the path.
                (None, None) => {}

                // One intersection; use that.
                (Some(solution), None) | (None, Some(solution)) => lookahead_point = solution,

                // Two intersections; pick whichever one is closest to the next point on the path.
                (Some(solution_1), Some(solution_2)) => {
                    lookahead_point = if solution_1.distance(next.position)
                        < solution_2.distance(next.position)
                    {
                        solution_1
                    } else {
                        solution_2
                    };
                }
            };

            // Take the profiled velocity of the closest point to the robot on the path.
            let velocity = if current.position.distance(position) < next.position.distance(position)
            {
                current.velocity
            } else {
                next.velocity
            };

            let curvature = Self::signed_arc_curvature(position, heading, lookahead_point);

            _ = drivetrain.motors.set_voltages(
                Voltages(
                    velocity * (2.0 + curvature * self.track_width) / 2.0,
                    velocity * (2.0 - curvature * self.track_width) / 2.0,
                )
                .normalized(Motor::V5_MAX_VOLTAGE),
            );
        }

        _ = drivetrain.motors.set_voltages((0.0, 0.0));
    }

    fn signed_arc_curvature(start: Vec2<f64>, start_angle: Angle, end: Vec2<f64>) -> f64 {
        let delta = end - start;
        let side = (start_angle.sin() * delta.x - start_angle.cos() * delta.y).signum();

        let a = -start_angle.tan();
        let c = start_angle.tan() * start.x - start.y;
        let x = (a * end.x + end.y + c).abs() / (a * a + 1.0).sqrt();
        let d = start.distance(end);

        side * ((2.0 * x) / (d * d))
    }

    /// Finds the intersection points between a line segment and a circle.
    fn line_segment_circle_intersections(
        center: Vec2<f64>,
        radius: f64,
        start: Vec2<f64>,
        end: Vec2<f64>,
    ) -> (Option<Vec2<f64>>, Option<Vec2<f64>>) {
        // Subtract the circle's center to offset the system to origin.
        let offset_1 = start - center;
        let offset_2 = end - center;

        let (dx, dy) = {
            let delta = offset_2 - offset_1;
            (delta.x, delta.y)
        };
        let dr = offset_1.distance(offset_2);
        let d = offset_1.cross(offset_2);
        let discriminant = (radius * radius) * (dr * dr) - (d * d);

        let mut solutions = (None, None);

        // If our discriminant is greater than or equal to 0, the line formed as a slope of
        // point_1 and point_2 intersects the circle at least once.
        if discriminant >= 0.0 {
            // https://mathworld.wolfram.com/Circle-LineIntersection.html
            let discriminant_sqrt = discriminant.sqrt();
            let dr_squared = dr * dr;

            let solution_1 = Vec2::new(
                (d * dy + dy.signum() * dx * discriminant_sqrt) / dr_squared,
                (-d * dx + dy.abs() * discriminant_sqrt) / dr_squared,
            ) + center;
            let solution_2 = Vec2::new(
                (d * dy - dy.signum() * dx * discriminant_sqrt) / dr_squared,
                (-d * dx - dy.abs() * discriminant_sqrt) / dr_squared,
            ) + center;

            let min_x = solution_1.x.min(solution_2.x);
            let max_x = solution_1.x.max(solution_2.x);
            let min_y = solution_1.y.min(solution_2.y);
            let max_y = solution_1.y.max(solution_2.y);

            // Find the bounded intersections.
            // solution_1 and solution_2 are assumed to be true when the line formed as a slope between point_1 and point_2
            // extends infinitely, however we only want to consider intersections that are part of a line segment *between*
            // point_1 and point_2.

            // Solution 1 intersects the circle within the bounds of point_1 and point_2
            if (solution_1.x >= min_x && solution_1.x <= max_x)
                && (solution_1.y >= min_y && solution_1.y <= max_y)
            {
                solutions.0 = Some(solution_1);
            }

            // Solution 2 intersects the circle within the bounds of point_1 and point_2
            if (solution_2.x >= min_x && solution_2.x <= max_x)
                && (solution_2.y >= min_y && solution_2.y <= max_y)
            {
                solutions.1 = Some(solution_2);
            }
        }

        solutions
    }
}
