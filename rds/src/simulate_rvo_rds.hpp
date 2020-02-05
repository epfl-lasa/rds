#ifndef SIMULATE_RVO_RDS
#define SIMULATE_RVO_RDS

#include "simulation.hpp"
#include "collision_point.hpp"

struct SimulateRvoRds
{
	SimulateRvoRds(Simulation* s_rvo) : s_rvo(s_rvo) { };

	void stepEuler(float dt);

	void createCollisionPointsInRobotFrame(std::vector<RDS::CollisionPoint>* collision_points);

	Simulation* s_rvo;
};

#endif