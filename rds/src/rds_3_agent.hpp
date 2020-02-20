#ifndef RDS_3_AGENT_HPP
#define RDS_3_AGENT_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include <vector>

struct RDS3CapsuleConfiguration
{
	RDS3CapsuleConfiguration() { };
	RDS3CapsuleConfiguration(float tau, float delta, float v_max,
		const Geometry2D::Capsule& robot_shape, const Geometry2D::Vec2& p_ref)
	: tau(tau), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float tau, delta, v_max;
	Geometry2D::Capsule robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS3CapsuleAgent
{
	RDS3CapsuleAgent(Geometry2D::Vec2 position, float orientation, const RDS3CapsuleConfiguration& config)
	: position(position), orientation(orientation), rds_3_configuration(config), last_step_p_ref_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects);

	Geometry2D::Vec2 position, last_step_p_ref_velocity;
	float orientation;
	const RDS3CapsuleConfiguration rds_3_configuration;

private:
	void getCircleObjectsInLocalFrame(const std::vector<AdditionalPrimitives2D::Circle>& objects,
		std::vector<AdditionalPrimitives2D::Circle>* objects_local);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local);
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global);
};

struct RDS3CircleConfiguration
{
	RDS3CircleConfiguration() { };
	RDS3CircleConfiguration(float tau, float delta, float v_max,
		const AdditionalPrimitives2D::Circle& robot_shape, const Geometry2D::Vec2& p_ref)
	: tau(tau), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float tau, delta, v_max;
	AdditionalPrimitives2D::Circle robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS3CircleAgent
{
	RDS3CircleAgent(Geometry2D::Vec2 position, float orientation, const RDS3CircleConfiguration& config)
	: position(position), orientation(orientation), rds_3_configuration(config), last_step_p_ref_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects,
		const std::vector<Geometry2D::Capsule>& capsule_objects,
		std::vector<AdditionalPrimitives2D::Circle>::size_type circle_index_to_skip = -1);

	Geometry2D::Vec2 position, last_step_p_ref_velocity;
	float orientation;
	const RDS3CircleConfiguration rds_3_configuration;

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