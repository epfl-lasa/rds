#ifndef RVO_3_AGENT_HPP
#define RVO_3_AGENT_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include "rds_4.hpp" // for struct MovingCircle
#include <vector>

struct MovingCapsule
{
	MovingCapsule() { };
	MovingCapsule(const Geometry2D::Capsule& capsule, const Geometry2D::Vec2& p_ref,
		const Geometry2D::Vec2& velocity_p_ref, float omega)
	: capsule(capsule), p_ref(p_ref), velocity_p_ref(velocity_p_ref), omega(omega) { }
	Geometry2D::Capsule capsule;
	Geometry2D::Vec2 p_ref, velocity_p_ref;
	float omega;
};

struct RVO3Configuration
{
	RVO3Configuration() { };
	RVO3Configuration(float tau, float delta, float v_max, float radius);
	float tau, delta, v_max, radius, v_max_sqrt_2;
};

struct RVO3Agent
{
	RVO3Agent(Geometry2D::Vec2 position, const RVO3Configuration& config)
	: position(position), rvo_configuration(config), last_step_velocity(0.f, 0.f)
	{ }

	void stepEuler(float dt, const Geometry2D::Vec2& v_nominal,
		const std::vector<MovingCircle>& objects_with_preferred_velocities,
		const MovingCapsule& robot_with_preferred_velocity, unsigned long int skip_index);

	Geometry2D::Vec2 position, last_step_velocity;
	RVO3Configuration rvo_configuration;
};

#endif