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
	OneIntersection(Vec2),

	/// The line is a secant line. It crosses the circle, intersecting at two points.
	TwoIntersections(Vec2, Vec2),
}

impl LineCircleIntersections {
	
	/// Compute the points of intersection between a line extending infinitely in both directions
	/// and a circle defined by a center and radius.
	/// 
	/// The result is returned as an instance of [`Self`], having either no intersections ([`Self::None`]),
	/// one intersection as a tangent line ([`Self::OneIntersection`]), or two intersections as a secant line ([`Self::TwoIntersections`]).
	fn compute(line: (Vec2, Vec2), circle: (Vec2, f64)) {
		let (start, end) = line;
		let (center, radius) = circle;
			
		let d = end - start;
		let f = line.0 - circle.1;
		
		let a = Vec2::new(d.x.powi(2), d.y.powi(2));
		let b = Vec2::new(d.x * f.x, d.y * f.y) * 2.0;
		let c = Vec2::new(f.x.powi(2), f.y.powi(2)) * circle.1.powi(2);
		// let discriminant = b
	}

	/// Compute the points of intersection between a line segment formed by two points
	/// and a circle defined by a center and radius.
	/// 
	/// The result is returned as an instance of [`Self`], having either no intersections ([`Self::None`]),
	/// one intersection ([`Self::OneIntersection`]), or two intersections ([`Self::TwoIntersections`]).
	/// 
	/// This differs from [`LineCircleIntersections::compute`] in that it performs a bounds check to ensure that
	/// the intersections are contained within the line segment, which has a defined start and endpoint.
	fn compute_bounded(line: (Vec2, Vec2), circle: (Vec2, f64)) {}
}