#ifndef SIMULATE_RVO_RDS
#define SIMULATE_RVO_RDS

#include "simulation.hpp"
#include "collision_point.hpp"

struct SimulateRvoRds
{
	SimulateRvoRds(Simulation* s_rvo) : s_rvo(s_rvo), tau(1.f), delta(0.05f) { };

	void stepEuler(float dt);

	void createCollisionPointsInRobotFrame(std::vector<RDS::CollisionPoint>* collision_points);

	Simulation* s_rvo;
	float tau, delta;
};

#endif