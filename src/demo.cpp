#include "geometry.hpp"
#include "gui.hpp"

#include <iostream>

int main()
{
	// create some points and half-planes
	std::vector<Geometry2D::Vec2> points;
	std::vector<Geometry2D::HalfPlane2> half_planes;
	points.push_back(Geometry2D::Vec2(1.0, 1.0));
	points.push_back(Geometry2D::Vec2(-1.0, 1.0));
	points.push_back(Geometry2D::Vec2(1.0, -1.0));
	points.push_back(Geometry2D::Vec2(-1.0, -1.0));
	half_planes.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5));
	half_planes.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5));
	bool outside_range;
	Geometry2D::Vec2 intersection = half_planes[0].boundaryIntersection(half_planes[1],
		&outside_range);
	if (!outside_range)
		points.push_back(intersection);
	// draw them in a GUI
	GUI gui(0.0, half_planes, points);
	return 0;
}