/* 
 * David Gonon modified this file.
 */

/*
 * Blocks.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/*
 * Example file showing a demo with 100 agents split in four groups initially
 * positioned in four corners of the environment. Each agent attempts to move to
 * other side of the environment through a narrow passage generated by four
 * obstacles. There is no roadmap to guide the agents around the obstacles.
 */


#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif

#include <cmath>
#include <cstdlib>

#include <vector>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <RVO.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif


#include <chrono>
#include <thread>
#include "geometry.hpp"
#include "capsule.hpp"
#include "gui.hpp"
#include "rds_2_agent.hpp"

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;
using Geometry2D::Capsule;

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
std::vector<Circle> gui_circles;
std::vector<Polygon> gui_polygons;
std::vector<Capsule> gui_capsules;

RDS2CapsuleAgent rds_2_agent(Vec2(0.f, -110.f), 0.f, RDS2CapsuleConfiguration(1.f,1.f,0.1f,3.f, Capsule(
	2.f, Vec2(0.f, 2.f), Vec2(0.f, -2.f)), Vec2(0.f, 1.f)));

void getRDS2AgentRefP(float* x, float* y)
{
	Vec2 v_result;
	rds_2_agent.transformVectorLocalToGlobal(rds_2_agent.rds_2_configuration.p_ref, &v_result);
	*x = rds_2_agent.position.x + v_result.x;
	*y = rds_2_agent.position.y + v_result.y;
}

int updateGUI(RVO::RVOSimulator *sim, GUI* gui)
{
	gui_circles.resize(sim->getNumAgents());
	for (size_t i = 0; i != sim->getNumAgents(); i++)
	{
		gui_circles[i].center.x = sim->getAgentPosition(i).x();
		gui_circles[i].center.y = sim->getAgentPosition(i).y();
		gui_circles[i].radius = sim->getAgentRadius(i);
	}

	Vec2 v_result;
	rds_2_agent.transformVectorLocalToGlobal(rds_2_agent.rds_2_configuration.robot_shape.center_a(), &v_result);
	Vec2 cap_a_global = rds_2_agent.position + v_result;
	rds_2_agent.transformVectorLocalToGlobal(rds_2_agent.rds_2_configuration.robot_shape.center_b(), &v_result);
	Vec2 cap_b_global = rds_2_agent.position + v_result;
	Capsule global_capsule(rds_2_agent.rds_2_configuration.robot_shape.radius(), cap_a_global, cap_b_global);
	gui_capsules.resize(0);
	gui_capsules.push_back(global_capsule);

	int gui_error = gui->update();
	// wait to have the specified cycle time (= 1/framerate)
	std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - t_gui_update));
	t_gui_update = std::chrono::high_resolution_clock::now();
	return gui_error;
}

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator *sim)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
	std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.25f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
	float p_ref_x, p_ref_y;
	getRDS2AgentRefP(&p_ref_x, &p_ref_y);
	sim->addAgent(RVO::Vector2(p_ref_x, p_ref_y), 15.0f, 10, 5.0f, 5.0f, 5.f, 2.0f);
	goals.push_back(RVO::Vector2(-p_ref_x, -p_ref_y));

	for (size_t i = 0; i < 5; ++i) {
		for (size_t j = 0; j < 5; ++j) {
			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, 75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, 75.0f));
		}
	}

	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
	/*
	obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
	obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	sim->addObstacle(obstacle3);
	sim->addObstacle(obstacle4);*/

	/* Process the obstacles so that they are accounted for in the simulation. */
	sim->processObstacles();
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim)
{
	/* Output the current global time. */
	std::cout << sim->getGlobalTime();

	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		std::cout << " " << sim->getAgentPosition(i);
	}

	std::cout << std::endl;
}
#endif

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
		                          dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 20.0f * 20.0f) {
			return false;
		}
	}

	return true;
}

int main()
{
	/* Create a new simulator instance. */
	RVO::RVOSimulator *sim = new RVO::RVOSimulator();

	/* Set up the scenario. */
	setupScenario(sim);

	GUI gui("Demo of RVO2 library (Blocks example)", 250.f);
	gui.circles = &gui_circles;
	gui.polygons = &gui_polygons;
	Polygon polygon;
	for (size_t i = 0; i < sim->getNumObstacleVertices(); i++)
	{
		polygon.push_back(Vec2(sim->getObstacleVertex(i).x(), sim->getObstacleVertex(i).y()));
		if (sim->getNextObstacleVertexNo(i) <= i)
		{
			gui_polygons.push_back(polygon);
			polygon.resize(0);
		}
	}
	gui.capsules = &gui_capsules;

	/* Perform (and manipulate) the simulation. */
	do {
//#if RVO_OUTPUT_TIME_AND_POSITIONS
//		updateVisualization(sim);
//#endif
		setPreferredVelocities(sim);

		float robot_x_sim = sim->getAgentPosition(0).x();
		float robot_y_sim = sim->getAgentPosition(0).y();

		std::vector<Circle> objects(sim->getNumAgents() - 1);
		for (size_t i = 1; i != sim->getNumAgents(); i++)
		{
			objects[i - 1].center.x = sim->getAgentPosition(i).x();
			objects[i - 1].center.y = sim->getAgentPosition(i).y();
			objects[i - 1].radius = sim->getAgentRadius(i);
		}
		
		Vec2 robot_v_preferred(sim->getAgentPrefVelocity(0).x(), sim->getAgentPrefVelocity(0).y());

		sim->doStep();

		float robot_x_sim_new = sim->getAgentPosition(0).x();
		float robot_y_sim_new = sim->getAgentPosition(0).y();

		float robot_v_x_sim = (robot_x_sim_new - robot_x_sim)/sim->getTimeStep();
		float robot_v_y_sim = (robot_y_sim_new - robot_y_sim)/sim->getTimeStep();

		//rds_2_agent.stepEuler(sim->getTimeStep(), Vec2(robot_v_x_sim, robot_v_y_sim), objects);
		rds_2_agent.stepEuler(sim->getTimeStep(), robot_v_preferred, objects);

		float p_ref_x, p_ref_y;
		getRDS2AgentRefP(&p_ref_x, &p_ref_y);

		sim->setAgentPosition(0, RVO::Vector2(p_ref_x, p_ref_y));
	}
	while (!reachedGoal(sim) && (updateGUI(sim, &gui) == 0));

	delete sim;

	return 0;
}
