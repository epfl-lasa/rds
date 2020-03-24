#ifndef RDS_4_AGENT_HPP
#define RDS_4_AGENT_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include "rds_4.hpp"
#include <vector>

struct RDSCapsuleConfiguration
{
	RDSCapsuleConfiguration() { };
	RDSCapsuleConfiguration(float tau, float delta, float v_max,
		const Geometry2D::Capsule& robot_shape, const Geometry2D::Vec2& p_ref)
	: tau(tau), delta(delta), v_max(v_max), robot_shape(robot_shape), p_ref(p_ref) { }
	float tau, delta, v_max;
	Geometry2D::Capsule robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct RDS4CapsuleAgent
{
	RDS4CapsuleAgent() { }
	RDS4CapsuleAgent(Geometry2D::Vec2 position, float orientation, const RDSCapsuleConfiguration& config)
	: position(position), orientation(orientation), rds_configuration(config), last_step_p_ref_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects);

	Geometry2D::Vec2 position, last_step_p_ref_velocity;
	float orientation;
	RDSCapsuleConfiguration rds_configuration;

private:
	void getObjectsInLocalFrame(const std::vector<MovingCircle>& objects,
		std::vector<MovingCircle>* objects_local);
public:
	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local) const;
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global) const;
};

#endif