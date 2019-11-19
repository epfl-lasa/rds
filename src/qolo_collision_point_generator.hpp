#ifndef QOLO_COLLISION_POINT_GENERATOR_HPP
#define QOLO_COLLISION_POINT_GENERATOR_HPP

#include "rds_wrap.hpp"
#include "geometry.hpp"

struct QoloCollisionPointGenerator : public CollisionPointGenerator
{
	// is not using correct location/orientation for the lrf/lidar
	QoloCollisionPointGenerator();

	Geometry2D::Vec2& lrf_location;
	float lrf_orientation;
};

#endif