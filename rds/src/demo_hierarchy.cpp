#include "geometry.hpp"
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;

#include "capsule.hpp"
using Geometry2D::Capsule;

#include "gui.hpp"
#include "rds_2_agent.hpp"

#include <RVO.h> // external official RVO2 library

#include <cmath>
#include <string>
#include <chrono>
#include <thread>

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

std::vector<RDS2CircleAgent> circle_agents;
std::vector<Circle> circle_objects_global;
std::vector<RDS2CapsuleAgent> capsule_agents;
std::vector<Capsule> capsule_objects_global;

std::vector<Circle> circle_objects_global_rvo_only;

/*
	for (auto& cap : capsule_agents)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global(cap.position + v_result);
		sim->addAgent(RVO::Vector2(p_ref_global.x, p_ref_global.y), 15.0f, 10, 5.0f, 5.0f,
			cap.rds_2_configuration.robot_shape.radius, cap.rds_2_configuration.v_max);
	}
	for (auto& cir : circle_agents)
	{
		Vec2 v_result;
		cir.transformVectorLocalToGlobal(cir.rds_2_configuration.robot_shape.center, &v_result);
		Vec2 center_global = cir.position + v_result;
		sim->addAgent(RVO::Vector2(center_global.x, center_global.y), 15.0f, 10, 5.0f, 5.0f,
			cir.rds_2_configuration.robot_shape.radius, cir.rds_2_configuration.v_max);
	}
*/

void update_objects_global()
{
	for (std::vector<Circle>::size_type i = 0; i != circle_objects_global.size(); i++)
	{
		Vec2 v_result;
		circle_agents[i].transformVectorLocalToGlobal(circle_agents[i].rds_2_configuration.robot_shape.center, &v_result);
		circle_objects_global[i].center = circle_agents[i].position + v_result;
	}
	for (std::vector<Capsule>::size_type i = 0; i != capsule_objects_global.size(); i++)
	{
		Vec2 v_result;
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_2_configuration.robot_shape.center_a, &v_result);
		Vec2 center_a(capsule_agents[i].position + v_result);
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_2_configuration.robot_shape.center_b, &v_result);
		Vec2 center_b(capsule_agents[i].position + v_result);
		capsule_objects_global[i] = Capsule(capsule_objects_global[i].radius, center_a, center_b);
	}
}

void update_RVO_agents_positions(RVO::RVOSimulator* sim)
{
	unsigned long int i = 0;
	for (auto& cap : capsule_agents)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global(cap.position + v_result);
		sim->setAgentPosition(i, RVO::Vector2(p_ref_global.x, p_ref_global.y));
		i++;
	}
	for (auto& cir : circle_objects_global)
	{
		sim->setAgentPosition(i, RVO::Vector2(cir.center.x, cir.center.y));
		i++;
	}
}

void set_RVO_agents_preferred_velocities(RVO::RVOSimulator* sim)
{
	for (unsigned long int i = 0; i != sim->getNumAgents(); i++)
	{
		if (i < capsule_agents.size())
		{
			RVO::Vector2 v_preferred(0.f, 1.2f);
			sim->setAgentPrefVelocity(i, v_preferred);
		}
		else
		{
			RVO::Vector2 v_preferred(1.2f, 0.f);//(0.f, -1.2f);
			sim->setAgentPrefVelocity(i, v_preferred);
		}
	}
}

int update_gui(GUI* gui)
{
	std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - t_gui_update));
	t_gui_update = std::chrono::high_resolution_clock::now();
	
	for (auto& cap : capsule_agents) // add the robot circle (serves for display only)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global(cap.position + v_result);
		circle_objects_global.push_back(Circle(p_ref_global, cap.rds_2_configuration.robot_shape.radius));
	}
	int gui_error = gui->update();
	for (auto& cap : capsule_agents)
		circle_objects_global.pop_back(); // remove again the robot circle (serves for display only)
	return gui_error;
}

void simulate(const char* title, float screen_size)
{
	GUI gui(title, screen_size);
	gui.circles = &circle_objects_global;
	gui.capsules = &capsule_objects_global;

	{
		circle_objects_global.resize(circle_agents.size());
		for (std::vector<Circle>::size_type i = 0; i != circle_objects_global.size(); i++)
			circle_objects_global[i].radius = circle_agents[i].rds_2_configuration.robot_shape.radius;
		capsule_objects_global.resize(capsule_agents.size());
		for (std::vector<Capsule>::size_type i = 0; i != capsule_objects_global.size(); i++)
			capsule_objects_global[i].radius = capsule_agents[i].rds_2_configuration.robot_shape.radius;
	}

	RVO::RVOSimulator sim;
	for (auto& cap : capsule_agents)
	{
		sim.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, 5.0f, 5.0f,
			cap.rds_2_configuration.robot_shape.radius + cap.rds_2_configuration.delta*cap.rds_2_configuration.D,
			cap.rds_2_configuration.v_max);
	}
	for (auto& cir : circle_agents)
	{
		sim.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, 5.0f, 5.0f,
			cir.rds_2_configuration.robot_shape.radius + cir.rds_2_configuration.delta*cir.rds_2_configuration.D,
			cir.rds_2_configuration.v_max);
	}

	GUI gui_rvo_only((std::string("RVO only: ") + std::string(title)).c_str(), screen_size);
	gui_rvo_only.circles = &circle_objects_global_rvo_only;

	update_objects_global();
	RVO::RVOSimulator sim_rvo_only;
	for (auto& cap : capsule_agents)
	{
		const Capsule& c(cap.rds_2_configuration.robot_shape);
		Vec2 circle_center_local = (c.center_a + c.center_b)/2.f;
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(circle_center_local, &v_result);
		float r = (c.center_a - c.center_b).norm()/2.f + c.radius + cap.rds_2_configuration.delta*cap.rds_2_configuration.D;
		sim_rvo_only.addAgent(RVO::Vector2((cap.position + v_result).x, (cap.position + v_result).y), 15.0f, 10, 5.0f, 5.0f,
			r, cap.rds_2_configuration.v_max);
		circle_objects_global_rvo_only.push_back(Circle(Vec2(0.f, 0.f), r));
	}
	for (unsigned long int i = 0; i < circle_agents.size(); i++)
	{
		float r = circle_agents[i].rds_2_configuration.robot_shape.radius +
			circle_agents[i].rds_2_configuration.delta*circle_agents[i].rds_2_configuration.D;
		sim_rvo_only.addAgent(RVO::Vector2(circle_objects_global[i].center.x, circle_objects_global[i].center.y),
			15.0f, 10, 5.0f, 5.0f, r, circle_agents[i].rds_2_configuration.v_max);
		circle_objects_global_rvo_only.push_back(Circle(Vec2(0.f, 0.f), r));
	}
	
	int n_iterations = 5;
	float dt = gui_cycle_time.count()*0.001f/n_iterations;
	sim_rvo_only.setTimeStep(dt*n_iterations);
	do
	{
		for (int k = 0; k != n_iterations; k++)
		{
			update_objects_global();

			update_RVO_agents_positions(&sim);
			set_RVO_agents_preferred_velocities(&sim);
			sim.doStep(); // this should set the velocity to read out for each agent

			unsigned long int i_sim = 0;
			for (std::vector<RDS2CapsuleAgent>::size_type i = 0; i != capsule_agents.size(); i++)
			{
				RVO::Vector2 v_nominal = sim.getAgentVelocity(i_sim);
				capsule_agents[i].stepEuler(dt, Vec2(v_nominal.x(), v_nominal.y()), circle_objects_global);
				i_sim++;
			}
			for (std::vector<RDS2CircleAgent>::size_type i = 0; i != circle_agents.size(); i++)
			{
				RVO::Vector2 v_nominal = sim.getAgentVelocity(i_sim);
				circle_agents[i].stepEuler(dt, Vec2(v_nominal.x(), v_nominal.y()), circle_objects_global, capsule_objects_global, i);
				i_sim++;
			}
		}
		for (std::vector<Circle>::size_type i = 0; i != circle_objects_global_rvo_only.size(); i++)
		{
			circle_objects_global_rvo_only[i].center = Vec2(sim_rvo_only.getAgentPosition(i).x(),
				sim_rvo_only.getAgentPosition(i).y());
		}
		set_RVO_agents_preferred_velocities(&sim_rvo_only);
		sim_rvo_only.doStep();
	}
	while ((update_gui(&gui) == 0) | (gui_rvo_only.update() == 0));
}

int main(int argc, char** argv)
{
	int simu_index = 0;
	if (argc > 1)
		simu_index = std::stoi(argv[1]);

	capsule_agents.push_back(RDS2CapsuleAgent(Vec2(0.2f, -5.f), 0.f, RDS2CapsuleConfiguration(1.f,1.f,0.05f,1.5f,
		Capsule(0.5f, Vec2(0.f, 0.2f), Vec2(0.f, -0.3f)), Vec2(0.f, 0.2f))));

	switch (simu_index)
	{
		case 0:
		{
			Vec2 p_ref(0.f, 1.f);
			/*circle_agents.push_back(RDS2CircleAgent(Vec2(0.f, 0.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
				Circle(p_ref, 0.5f), p_ref)));
			circle_agents.push_back(RDS2CircleAgent(Vec2(2.f, 2.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
				Circle(p_ref, 0.5f), p_ref)));
			circle_agents.push_back(RDS2CircleAgent(Vec2(0.2f, 2.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
				Circle(p_ref, 0.5f), p_ref)));
			circle_agents.push_back(RDS2CircleAgent(Vec2(0.5f, 1.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
				Circle(p_ref, 0.5f), p_ref)));
			circle_agents.push_back(RDS2CircleAgent(Vec2(0.3f, 1.5f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
				Circle(p_ref, 0.5f), p_ref)));*/

			//circle_agents.push_back(RDS2CircleAgent(Vec2(-5.5f, 0.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f,
			//	Circle(p_ref, 0.5f), p_ref)));

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					circle_agents.push_back(RDS2CircleAgent(Vec2(-20.f + i*2.7f + (j%2)*1.5f, 0.f + j*2.7f),//(-4.f + i*2.7f + (j%2)*1.5f, 8.f + j*2.7f),
						0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f, Circle(p_ref, 0.5f), p_ref)));
				}
			}

			simulate("Hierarchical RVO-RDS control", 24.f);
			if (argc > 2)
				break;
		}
	}
}