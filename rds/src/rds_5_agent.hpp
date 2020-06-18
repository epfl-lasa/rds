#ifndef RDS_5_AGENT_HPP
#define RDS_5_AGENT_HPP

#include "geometry.hpp"
#include "rds_5.hpp"
#include "capsule.hpp"
#include "vw_limits.hpp"
#include <vector>

struct RDS5CapsuleConfiguration
{
	RDS5CapsuleConfiguration() { };
	RDS5CapsuleConfiguration(float tau, float delta, float y_p_ref,
		float linear_acceleration_limit, float angular_acceleration_limit, float dt_cycle,
		const VWDiamond& vw_diamond_limits, const Geometry2D::Capsule& robot_shape)
	: tau(tau), delta(delta), y_p_ref(y_p_ref), linear_acceleration_limit(linear_acceleration_limit),
	angular_acceleration_limit(angular_acceleration_limit), dt_cycle(dt_cycle),
	vw_diamond_limits(vw_diamond_limits), robot_shape(robot_shape), p_ref(0.f, y_p_ref) { }

	float tau, delta, y_p_ref, linear_acceleration_limit, angular_acceleration_limit, dt_cycle;
	VWDiamond vw_diamond_limits;
	Geometry2D::Capsule robot_shape;

	Geometry2D::Vec2 p_ref;
};

struct RDS5CapsuleAgent
{
	RDS5CapsuleAgent() { }
	RDS5CapsuleAgent(Geometry2D::Vec2 position, float orientation, const RDS5CapsuleConfiguration& config)
	: position(position), orientation(orientation), rds_configuration(config), last_step_p_ref_velocity(0.f, 0.f),
	last_step_p_ref_velocity_local(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects);

	Geometry2D::Vec2 position, last_step_p_ref_velocity, last_step_p_ref_velocity_local;
	float orientation;
	RDS5CapsuleConfiguration rds_configuration;

private:
	void getObjectsInLocalFrame(const std::vector<MovingCircle>& objects,
		std::vector<MovingCircle>* objects_local);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local) const;
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global) const;
	void transformReferencePointVelocityToPointVelocity(const Geometry2D::Vec2& p_local, const Geometry2D::Vec2& v_p_ref_global,
		Geometry2D::Vec2* v_global) const;
};

#endif