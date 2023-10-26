use alloc::vec::Vec;
use num_traits::real::Real;

use crate::math::Vec2;

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum LineCircleIntersections {
	/// The line segment does not intersect with the circle.
	#[default]
	None,

	/// The line is a tangent line. It touches the circle's edge exactly once, and
	/// therefore has one intersection. 
	Tangent(Vec2),

	/// The line is a secant line. It crosses the circle, intersecting at two points.
	Secant(Vec2, Vec2),
}

impl LineCircleIntersections {
	
	/// Compute the points of intersection between a line extending infinitely in both directions
	/// and a circle defined by a center and radius.
	/// 
	/// The result is returned as an instance of [`Self`], having either no intersections ([`Self::None`]),
	/// one intersection as a tangent line ([`Self::OneIntersection`]), or two intersections as a secant line ([`Self::TwoIntersections`]).
	pub fn compute(line: (Vec2, Vec2), circle: (Vec2, f64)) -> Self {
		let (start, end) = line;
		let (center, radius) = circle;

		let offset_1 = start - center;
		let offset_2 = end - center;

		let dx = offset_2.x - offset_1.x;
		let dy = offset_2.y - offset_1.y;
		let dr = offset_1.distance(offset_2);
		let d = offset_1.cross(offset_2);
		let discriminant = radius.powi(2) * dr.powi(2) - d.powi(2);

		if discriminant >= 0.0 {
			let solution_1 = Vec2::new(
				(d * dy + dy.signum() * dx * discriminant.sqrt()) / dr.powi(2),
				(-d * dx + dy.abs() * discriminant.sqrt()) / dr.powi(2)
			) + center;
			let solution_2 = Vec2::new(
				(d * dy - dy.signum() * dx * discriminant.sqrt()) / dr.powi(2),
				(-d * dx - dy.abs() * discriminant.sqrt()) / dr.powi(2)
			);

			
		}
		
		Self::None
	}

	/// Compute the points of intersection between a line segment formed by two points
	/// and a circle defined by a center and radius.
	/// 
	/// The result is returned as an instance of [`Self`], having either no intersections ([`Self::None`]),
	/// one intersection ([`Self::OneIntersection`]), or two intersections ([`Self::TwoIntersections`]).
	/// 
	/// This differs from [`LineCircleIntersections::compute`] in that it performs a bounds check to ensure that
	/// the intersections are contained within the line segment, which has a defined start and endpoint.
	pub fn compute_bounded(line: (Vec2, Vec2), circle: (Vec2, f64)) -> Self {
		Self::None
	}
}