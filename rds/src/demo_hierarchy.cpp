#include "geometry.hpp"
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;

#include "capsule.hpp"
using Geometry2D::Capsule;

#include "gui.hpp"
#include "rds_2_agent.hpp"

#include <RVO.h> // external official RVO2 library

#include <string>
#include <chrono>
#include <thread>

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

std::vector<RDS2CircleAgent> circle_agents;
std::vector<Circle> circle_objects_global;
std::vector<RDS2CapsuleAgent> capsule_agents;
std::vector<Capsule> capsule_objects_global;

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

void simulate(const char* title, float screen_size)
{
	GUI gui(title, screen_size);
	gui.circles = &circle_objects_global;
	gui.capsules = &capsule_objects_global;

	circle_objects_global.resize(circle_agents.size());
	for (std::vector<Circle>::size_type i = 0; i != circle_objects_global.size(); i++)
		circle_objects_global[i].radius = circle_agents[i].rds_2_configuration.robot_shape.radius;
	capsule_objects_global.resize(capsule_agents.size());
	for (std::vector<Capsule>::size_type i = 0; i != capsule_objects_global.size(); i++)
		capsule_objects_global[i].radius = capsule_agents[i].rds_2_configuration.robot_shape.radius;
	
	int n_iterations = 5;
	float dt = gui_cycle_time.count()*0.001f/n_iterations;
	do
	{
		for (int k = 0; k != n_iterations; k++)
		{
			update_objects_global();
			for (std::vector<RDS2CircleAgent>::size_type i = 0; i != circle_agents.size(); i++)
				circle_agents[i].stepEuler(dt, Vec2(0.f, -1.f), circle_objects_global, capsule_objects_global, i);
			for (std::vector<RDS2CapsuleAgent>::size_type i = 0; i != capsule_agents.size(); i++)
				capsule_agents[i].stepEuler(dt, Vec2(0.f, 1.f), circle_objects_global);
		}
		
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
	}
	while (gui.update() == 0);
}

int main(int argc, char** argv)
{
	int simu_index = 0;
	if (argc > 1)
		simu_index = std::stoi(argv[1]);

	capsule_agents.push_back(RDS2CapsuleAgent(Vec2(0.2f, -5.f), 0.f, RDS2CapsuleConfiguration(1.f,1.f,0.05f,1.5f, Capsule(
		0.5f, Vec2(0.f, 0.2f), Vec2(0.f, -0.3f)), Vec2(0.f, 0.1f))));

	switch (simu_index)
	{
		case 0:
		{
			circle_agents.push_back(RDS2CircleAgent(Vec2(0.f, 0.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f, Circle(
				Vec2(0.f, 1.f), 0.5f), Vec2(0.f, 1.f))));
			circle_agents.push_back(RDS2CircleAgent(Vec2(2.f, 2.f), 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.5f, Circle(
				Vec2(0.f, 1.f), 0.5f), Vec2(0.f, 1.f))));

			simulate("Hierarchical RVO-RDS control", 18.f);
			if (argc > 2)
				break;
		}
	}
}