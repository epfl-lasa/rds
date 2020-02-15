#ifndef SIMULATE_RVO_RDS
#define SIMULATE_RVO_RDS

#include "simulation.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"
#include "capsule.hpp"

#include <vector>

struct RDS2Configuration
{
	float T, D, delta, v_max;
	Geometry2D::Capsule robot_shape;
	Geometry2D::Vec2 p_ref;
};

struct SimulateRvoRds
{
	SimulateRvoRds(Simulation* s_rvo) : s_rvo(s_rvo), tau(1.f), delta(0.05f) { };

	void stepEuler(float dt);

	void createCollisionPointsInRobotFrame(std::vector<RDS::CollisionPoint>* collision_points);

	Simulation* s_rvo;
	float tau, delta;

	std::vector<Geometry2D::HalfPlane2> rds_constraints;

	void stepEulerRDS2(float dt, const RDS2Configuration& rds_2_config);

	void getObjectsInLocalFrame(std::vector<AdditionalPrimitives2D::Circle>* objects);
};

#endif