#ifndef RDS_2_AGENT_HPP
#define RDS_2_AGENT_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include <vector>

struct RDS2CapsuleConfiguration
{
	RDS2CapsuleConfiguration() { };
	RDS2CapsuleConfiguration(float T, float D, float delta, float v_max,
		const Geometry2D::Capsule& robot_shape, const Geometry2D::Vec2& p_ref)
	: T(T), D(D), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float T, D, delta, v_max;
	Geometry2D::Capsule robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS2CapsuleAgent
{
	RDS2CapsuleAgent(Geometry2D::Vec2 position, float orientation, const RDS2CapsuleConfiguration& config)
	: position(position), orientation(orientation), rds_2_configuration(config), last_step_p_ref_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects);
		//, const std::vector<AdditionalPrimitives2D::Polygon>& convex_polygon_objects);

	Geometry2D::Vec2 position, last_step_p_ref_velocity;
	float orientation;
	const RDS2CapsuleConfiguration rds_2_configuration;

	//struct PolygonException { };
private:
	/*void getClosestPointOfConvexPolygon(const AdditionalPrimitives2D::Polygon& convex_polygon,
		const Geometry2D::Vec2& pt_query, Geometry2D::Vec2* closest_point);*/

	void getCircleObjectsInLocalFrame(const std::vector<AdditionalPrimitives2D::Circle>& objects,
		std::vector<AdditionalPrimitives2D::Circle>* objects_local);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local);
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global);
};

struct RDS2CircleConfiguration
{
	RDS2CircleConfiguration() { };
	RDS2CircleConfiguration(float T, float D, float delta, float v_max,
		const AdditionalPrimitives2D::Circle& robot_shape, const Geometry2D::Vec2& p_ref)
	: T(T), D(D), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float T, D, delta, v_max;
	AdditionalPrimitives2D::Circle robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS2CircleAgent
{
	RDS2CircleAgent(Geometry2D::Vec2 position, float orientation, const RDS2CircleConfiguration& config)
	: position(position), orientation(orientation), rds_2_configuration(config), last_step_p_ref_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects,
		const std::vector<Geometry2D::Capsule>& capsule_objects,
		std::vector<AdditionalPrimitives2D::Circle>::size_type circle_index_to_skip = -1);

	Geometry2D::Vec2 position, last_step_p_ref_velocity;
	float orientation;
	const RDS2CircleConfiguration rds_2_configuration;

private:
	void getCircleObjectsInLocalFrame(const std::vector<AdditionalPrimitives2D::Circle>& circle_objects,
		std::vector<AdditionalPrimitives2D::Circle>* circle_objects_local,
		std::vector<AdditionalPrimitives2D::Circle>::size_type circle_index_to_skip);
	void getCapsuleObjectsInLocalFrame(const std::vector<Geometry2D::Capsule>& capsule_objects,
		std::vector<Geometry2D::Capsule>* capsule_objects_local);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local);
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global);
};

#endif