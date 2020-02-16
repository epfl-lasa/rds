#ifndef RDS_2_AGENT_HPP
#define RDS_2_AGENT_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include <vector>

struct RDS2Configuration
{
	RDS2Configuration() { };
	RDS2Configuration(float T, float D, float delta, float v_max,
		const Geometry2D::Capsule& robot_shape, const Geometry2D::Vec2& p_ref)
	: T(T), D(D), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float T, D, delta, v_max;
	Geometry2D::Capsule robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS2Agent
{
	RDS2Agent(Geometry2D::Vec2 position, float orientation, const RDS2Configuration& config)
	: position(position), orientation(orientation), rds_2_configuration(config) { }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects);
		//, const std::vector<AdditionalPrimitives2D::Polygon>& convex_polygon_objects);

	Geometry2D::Vec2 position;
	float orientation;
	const RDS2Configuration rds_2_configuration;

	//struct PolygonException { };
private:
	/*void getClosestPointOfConvexPolygon(const AdditionalPrimitives2D::Polygon& convex_polygon,
		const Geometry2D::Vec2& pt_query, Geometry2D::Vec2* closest_point);*/

	void getCircleObjectsInLocalFrame(std::vector<AdditionalPrimitives2D::Circle>* objects);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local);
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global);
};

#endif