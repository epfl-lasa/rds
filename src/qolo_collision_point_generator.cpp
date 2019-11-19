#include "qolo_collision_point_generator.hpp"
#include "geometry.hpp"

#include <cmath>

QoloCollisionPointGenerator::QoloCollisionPointGenerator()
	: lrf_location(Geometry2D::Vec2(0.f, 0.f))
	, lrf_orientation(0.f)
{
	// create a Qolo-like shape using 8 circles
	float scale = 0.3f*2.f/357.0f*2.f;

	Geometry2D::Vec2 position;
	float radius;

	position.x = 208.391*scale;
	position.y = 95.028*scale;
	radius = 77.637*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -208.391*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 153.585*scale;
	position.y = 4.019*scale;
	radius = 136.843*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -153.585*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 57.833*scale;
	position.y = -105.522*scale;
	radius = 227.437*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -57.833*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 0.0;
	position.y = -284.338*scale;
	radius = 230.051*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 0.0;
	position.y = -383.821*scale;
	radius = 174.055*scale;
	robot_shape_circles->push_back(AdditionalPrimitives2D::Circle(position, radius));
}
