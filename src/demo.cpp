#include "geometry.hpp"
#include "gui.hpp"

#include <iostream>
#include <cmath>

int main()
{
	// create some points, half-planes and circles
	std::vector<Geometry2D::Vec2> points;
	std::vector<Geometry2D::HalfPlane2> half_planes;
	std::vector<Geometry2D::Circle2> circles;
	points.push_back(Geometry2D::Vec2(1.0, 1.0));
	points.push_back(Geometry2D::Vec2(-1.0, 1.0));
	points.push_back(Geometry2D::Vec2(1.0, -1.0));
	points.push_back(Geometry2D::Vec2(-1.0, -1.0));
	half_planes.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5));
	half_planes.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5));
	Geometry2D::Vec2 v(-0.5, -1.0);
	half_planes.push_back(Geometry2D::HalfPlane2(v/v.norm(), 0.4));
	circles.push_back(Geometry2D::Circle2(Geometry2D::Vec2(), 1.0));
	// compute half-plane boundary intersections
	bool outside_range;
	Geometry2D::Vec2 intersection_1 = half_planes[0].boundaryIntersection(half_planes[1],
		&outside_range);
	if (!outside_range)
		points.push_back(intersection_1);
	Geometry2D::Vec2 intersection_2 = half_planes[0].boundaryIntersection(half_planes[2],
		&outside_range);
	if (!outside_range)
		points.push_back(intersection_2);
	Geometry2D::Vec2 intersection_3 = half_planes[1].boundaryIntersection(half_planes[2],
		&outside_range);
	if (!outside_range)
		points.push_back(intersection_3);
	// draw the incircle of the triangle defined by the three intersection points above
	Geometry2D::Vec2 center;
	float radius = incircleRadius(intersection_1, intersection_2, intersection_3, &center);
	circles.push_back(Geometry2D::Circle2(center, radius));

	// draw them in a GUI
	GUI gui(0.0, half_planes, points, circles);
	return 0;
}