#include "geometry.hpp"
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;

#include "capsule.hpp"
using Geometry2D::Capsule;

#include "gui.hpp"
#include "rds_2_agent.hpp"

#include <RVO.h> // external official RVO2 library

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <chrono>
#include <thread>
#include <random>
#include <iostream>

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

std::vector<RDS2CircleAgent> circle_agents;
std::vector<Circle> circle_objects_global;
std::vector<RDS2CapsuleAgent> capsule_agents;
std::vector<Capsule> capsule_objects_global;

std::vector<Circle> circle_objects_global_rvo_only;

std::vector<RVO::Vector2> all_v_preferred;

std::default_random_engine generator(1);

unsigned long int n_steps = 0;
const float speed_measurement_radius = 5.f;
float average_speed_around_robot_hierarchical = 0.f;
float average_speed_around_robot_rvo_only = 0.f;
float average_forward_velocity_around_robot_hierarchical = 0.f;
float average_forward_velocity_around_robot_rvo_only = 0.f;

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

Vec2 normalized(const RVO::Vector2& v)
{
	return (Vec2(v.x(), v.y())).normalized();
}

Vec2 toVec2(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

void update_speed_measurements(RVO::RVOSimulator* sim, RVO::RVOSimulator* sim_rvo_only)
{
	Vec2 v_result;
	capsule_agents[0].transformVectorLocalToGlobal(capsule_agents[0].rds_2_configuration.p_ref, &v_result);
	Vec2 p_ref_robot = capsule_agents[0].position + v_result;

	unsigned int n_agents_hierarchical = 0;
	float speed_avrg_now_hierarchical = 0.f;
	float v_f_avrg_now_hier = 0.f;
	unsigned long int sim_i = 0;
	for (auto& cap : capsule_agents)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global = cap.position + v_result;
		if (((p_ref_robot - p_ref_global).norm() < speed_measurement_radius) && (cap.rds_2_configuration.v_max != 0.f))
		{
			speed_avrg_now_hierarchical =
				(n_agents_hierarchical*speed_avrg_now_hierarchical + cap.last_step_p_ref_velocity.norm())/double(n_agents_hierarchical + 1);
			float v_forward = cap.last_step_p_ref_velocity.dot(normalized(all_v_preferred[sim_i]));
			v_f_avrg_now_hier = (n_agents_hierarchical*v_f_avrg_now_hier + v_forward)/double(n_agents_hierarchical + 1);
			n_agents_hierarchical++;
		}
		sim_i++;
	}
	for (auto& cir : circle_agents)
	{
		Vec2 v_result;
		cir.transformVectorLocalToGlobal(cir.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global = cir.position + v_result;
		if (((p_ref_robot - p_ref_global).norm() < speed_measurement_radius) && (cir.rds_2_configuration.v_max != 0.f))
		{
			speed_avrg_now_hierarchical =
				(n_agents_hierarchical*speed_avrg_now_hierarchical + cir.last_step_p_ref_velocity.norm())/double(n_agents_hierarchical + 1);
			float v_forward = cir.last_step_p_ref_velocity.dot(normalized(all_v_preferred[sim_i]));
			v_f_avrg_now_hier = (n_agents_hierarchical*v_f_avrg_now_hier + v_forward)/double(n_agents_hierarchical + 1);
			n_agents_hierarchical++;
		}
		sim_i++;
	}
	average_speed_around_robot_hierarchical =
		(n_steps*average_speed_around_robot_hierarchical + speed_avrg_now_hierarchical)/double(n_steps + 1);
	average_forward_velocity_around_robot_hierarchical = (n_steps*average_forward_velocity_around_robot_hierarchical +
		v_f_avrg_now_hier)/double(n_steps + 1);

	Vec2 p_robot(sim_rvo_only->getAgentPosition(0).x(), sim_rvo_only->getAgentPosition(0).y());
	unsigned int n_agents_rvo_only = 0;
	float speed_avrg_now_rvo_only = 0.f;
	float v_f_avrg_now_rvoo = 0.f;
	for (unsigned long int i = 0; i != sim_rvo_only->getNumAgents(); i++)
	{
		Vec2 p_global(sim_rvo_only->getAgentPosition(i).x(), sim_rvo_only->getAgentPosition(i).y());
		if (((p_global - p_robot).norm()  < speed_measurement_radius) && (sim_rvo_only->getAgentMaxSpeed(i) != 0.f))
		{
			float speed = (Vec2(sim_rvo_only->getAgentVelocity(i).x(), sim_rvo_only->getAgentVelocity(i).y())).norm();
			speed_avrg_now_rvo_only = (n_agents_rvo_only*speed_avrg_now_rvo_only + speed)/double(n_agents_rvo_only + 1);
			float v_forward = (toVec2(sim_rvo_only->getAgentVelocity(i))).dot(normalized(all_v_preferred[i]));
			v_f_avrg_now_rvoo = (n_agents_rvo_only*v_f_avrg_now_rvoo + v_forward)/double(n_agents_rvo_only + 1);
			n_agents_rvo_only++;
		}
	}
	average_speed_around_robot_rvo_only = 
		(n_steps*average_speed_around_robot_rvo_only + speed_avrg_now_rvo_only)/double(n_steps + 1);
	average_forward_velocity_around_robot_rvo_only = (n_steps*average_forward_velocity_around_robot_rvo_only +
		v_f_avrg_now_rvoo)/double(n_steps + 1);
	n_steps++;
}

void generate_all_v_preferred()
{
	std::normal_distribution<float> normal_distribution (0.0,2.0);
	for (auto& cap : capsule_agents)
	{
		RVO::Vector2 v_preferred(1.2f, 0.f);
		all_v_preferred.push_back(v_preferred);
	}
	for (auto& cir : circle_agents)
	{	
		float sign = 1.f;
		if (normal_distribution(generator) > 0.f)
			sign = -1.f;
		RVO::Vector2 v_preferred(-1.2f + std::min(0.4f, std::max(-0.4f, 
			float(normal_distribution(generator)))), 0.f);
		all_v_preferred.push_back(v_preferred*sign);
	}
}

void shift_all_agents(RVO::RVOSimulator* sim, RVO::RVOSimulator* sim_rvo_only)
{
	Vec2 shift(-1.f*capsule_agents[0].position.x, 0.f);
	for (auto& cap : capsule_agents)
		cap.position = cap.position + shift;
	for (auto& cir : circle_agents)
		cir.position = cir.position + shift;
	RVO::Vector2 shift_rvo_only(-1.f*sim_rvo_only->getAgentPosition(0).x(), 0.f);
	for (unsigned long int i = 0; i != sim_rvo_only->getNumAgents(); i++)
	{
		RVO::Vector2 new_position = sim_rvo_only->getAgentPosition(i) + shift_rvo_only;
		sim_rvo_only->setAgentPosition(i, new_position);
	}
}

void reinsert_agents(float window_size, float corridor_width,  RVO::RVOSimulator* sim, RVO::RVOSimulator* sim_rvo_only)
{
	std::uniform_real_distribution<float> uniform_distribution (-1.0,1.0);
	float offset = 1.f;
	for (auto& cir : circle_agents)
	{
		if (cir.position.x < -window_size/2.f - offset)
		{
			cir.position.x = window_size/2.f;
			if (cir.rds_2_configuration.v_max != 0.f)
				cir.position.y = (corridor_width -2.f)/2.f*float(uniform_distribution(generator));
		}
		else if (cir.position.x > window_size/2.f + offset)
		{
			cir.position.x = -window_size/2.f;
			if (cir.rds_2_configuration.v_max != 0.f)
				cir.position.y = (corridor_width -2.f)/2.f*float(uniform_distribution(generator));
		}
	}
	for (unsigned long int i = 0; i != sim_rvo_only->getNumAgents(); i++)
	{
		RVO::Vector2 position = sim_rvo_only->getAgentPosition(i);
		if (position.x() < -window_size/2.f - offset)
		{
			if (sim_rvo_only->getAgentMaxSpeed(i) != 0.f)
			{
				sim_rvo_only->setAgentPosition(i, RVO::Vector2(window_size/2.f, 
					(corridor_width -2.f)/2.f*float(uniform_distribution(generator))));	
			}
			else
				sim_rvo_only->setAgentPosition(i, RVO::Vector2(window_size/2.f, position.y()));
		}
		else if (position.x() > window_size/2.f + offset)
		{
			if (sim_rvo_only->getAgentMaxSpeed(i) != 0.f)
			{
				sim_rvo_only->setAgentPosition(i, RVO::Vector2(-window_size/2.f, 
					(corridor_width -2.f)/2.f*float(uniform_distribution(generator))));
			}
			else
				sim_rvo_only->setAgentPosition(i, RVO::Vector2(-window_size/2.f, position.y()));
		}
	}
}

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
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_2_configuration.robot_shape.center_a(), &v_result);
		Vec2 center_a(capsule_agents[i].position + v_result);
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_2_configuration.robot_shape.center_b(), &v_result);
		Vec2 center_b(capsule_agents[i].position + v_result);
		capsule_objects_global[i] = Capsule(capsule_objects_global[i].radius(), center_a, center_b);
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

void set_RVO_agents_preferred_velocities(RVO::RVOSimulator* sim, int simu_index)
{
	switch (simu_index)
	{
		case 0:
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
			break;
		}
		case 1:
		{
			for (unsigned long int i = 0; i != sim->getNumAgents(); i++)
			{
				if (i == 1)
				{
					sim->setAgentPrefVelocity(i, RVO::Vector2(0.f, 0.f));
					continue;
				}
				Vec2 position(sim->getAgentPosition(i).x(), sim->getAgentPosition(i).y());
				Vec2 velocity((Vec2(-position.y, position.x)).normalized()*1.2f);
				if (i == 0)
					velocity = -1.f*velocity;
				velocity = velocity - 0.5f*std::max(0.f, position.norm() - 60.f)*position.normalized();
				sim->setAgentPrefVelocity(i, RVO::Vector2(velocity.x, velocity.y));
			}
			break;
		}
		case 2:
		{
			for (unsigned long int i = 0; i != sim->getNumAgents(); i++)
				sim->setAgentPrefVelocity(i, all_v_preferred[i]);
			break;
		}
	}
}

bool update_gui(GUI* gui, GUI* gui_rvo_only)
{
	std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - t_gui_update));
	t_gui_update = std::chrono::high_resolution_clock::now();
	
	for (auto& cap : capsule_agents) // add the robot circle (serves for display only)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_2_configuration.p_ref, &v_result);
		Vec2 p_ref_global(cap.position + v_result);
		circle_objects_global.push_back(Circle(p_ref_global, cap.rds_2_configuration.robot_shape.radius()));
	}
	bool still_open = (gui->update() == 0) | (gui_rvo_only->update() == 0);
	for (auto& cap : capsule_agents)
		circle_objects_global.pop_back(); // remove again the robot circle (serves for display only)
	return still_open;
}

void simulate(const char* title, float screen_size, int simu_index, float corridor_width = 0)
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
			capsule_objects_global[i] = Capsule(capsule_agents[i].rds_2_configuration.robot_shape.radius(), Vec2(0.f, -1.f), Vec2(0.f, 1.f));
	}

	RVO::RVOSimulator sim;
	for (auto& cap : capsule_agents)
	{
		sim.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, 5.0f, 5.0f,
			cap.rds_2_configuration.robot_shape.radius() + cap.rds_2_configuration.delta*cap.rds_2_configuration.D,
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
		Vec2 circle_center_local = (c.center_a() + c.center_b())/2.f;
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(circle_center_local, &v_result);
		float r = (c.center_a() - c.center_b()).norm()/2.f + c.radius() + cap.rds_2_configuration.delta*cap.rds_2_configuration.D;
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
			set_RVO_agents_preferred_velocities(&sim, simu_index);
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
		set_RVO_agents_preferred_velocities(&sim_rvo_only, simu_index);
		sim_rvo_only.doStep();

		update_speed_measurements(&sim, &sim_rvo_only);

		if (simu_index == 2)
		{
			shift_all_agents(&sim, &sim_rvo_only);
			reinsert_agents(screen_size, corridor_width, &sim, &sim_rvo_only);
		}
	}
	while (update_gui(&gui, &gui_rvo_only));
}

int main(int argc, char** argv)
{
	int simu_index = 0;
	if (argc > 1)
		simu_index = std::stoi(argv[1]);

	switch (simu_index)
	{
		case 0:
		{
			capsule_agents.push_back(RDS2CapsuleAgent(Vec2(0.2f, -5.f), 0.f, RDS2CapsuleConfiguration(1.f,1.f,0.05f,1.5f,
				Capsule(0.5f, Vec2(0.f, 0.2f), Vec2(0.f, -0.3f)), Vec2(0.f, 0.2f))));

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

			simulate("Hierarchical RVO-RDS control", 24.f, simu_index);
			//if (argc > 2)
				break;
		}
		case 1: //unfinished
		{
			capsule_agents.push_back(RDS2CapsuleAgent(Vec2(40.f, 40.f), 0.f, RDS2CapsuleConfiguration(1.f,1.f,0.05f,1.5f,
				Capsule(0.5f, Vec2(0.f, 0.2f), Vec2(0.f, -0.3f)), Vec2(0.f, 0.2f))));

			Vec2 p_ref(0.f, 1.f);
			// static agent (circular wall)
			float inner_radius = 50.f;
			circle_agents.push_back(RDS2CircleAgent(Vec2(0.f, 0.f) - p_ref, 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,0.f,
				Circle(p_ref, inner_radius), p_ref)));
		
			float pedestrian_radius = 0.5f;
			float density = 0.25f;
			float outer_radius = 60.f;
			float ring_surface = M_PI*(outer_radius*outer_radius - inner_radius*inner_radius);
			int n_pedestrians = int(ring_surface*density) - 1;

			std::default_random_engine generator(1);
			std::normal_distribution<double> normal_distribution (0.0,1.0);
			std::uniform_real_distribution<double> uniform_distribution (-1.0,1.0);

			for (int i = 0; i < n_pedestrians; i++)
			{
				while (true)
				{
					Vec2 position_candidate(uniform_distribution(generator), uniform_distribution(generator));
					position_candidate = outer_radius*position_candidate;
					bool valid = true;
					if (position_candidate.norm() > outer_radius)
						valid = false;
					if (position_candidate.norm() < inner_radius + pedestrian_radius + 1.f)
					for (auto& a : circle_agents)
					{
						if (!valid)
							break;
						Vec2 v_result;
						a.transformVectorLocalToGlobal(a.rds_2_configuration.p_ref, &v_result);
						if ((a.position + v_result - position_candidate - Vec2(0.f,1.f)).norm() <
							a.rds_2_configuration.robot_shape.radius + pedestrian_radius +
							a.rds_2_configuration.delta*a.rds_2_configuration.D)
						{
							valid = false;
							break;
						}
					}
					if (valid)
					{
						circle_agents.push_back(RDS2CircleAgent(position_candidate - p_ref, 0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,0.f,
							Circle(p_ref, pedestrian_radius), p_ref)));
						break;
					}
				}
			}

			simulate("Hierarchical RVO-RDS control", 120.f, simu_index);
			//if (argc > 2)
				break;
		}
		case 2:
		{
			float capsule_radius = 0.5f;
			float reference_point_y = 0.2f;
			float capsule_segment_length = 0.5f;
			capsule_agents.push_back(RDS2CapsuleAgent(Vec2(0.f, 0.f), -M_PI/2.f, RDS2CapsuleConfiguration(1.f,1.f,0.05f,1.5f,
				Capsule(capsule_radius, Vec2(0.f, reference_point_y),
					Vec2(0.f, reference_point_y-capsule_segment_length)),
				Vec2(0.f, reference_point_y))));

			float window_size = 50.f;
			float corridor_width = 20.f;
			float pillar_radius = 0.5f;
			float pedestrian_radius = 0.4f;
			float density = 0.15f;
			float bin_width = std::sqrt(1.f/density);

			Vec2 p_ref(0.f, 1.f);
			// moving agents
			float no_go_radius = capsule_segment_length + capsule_radius;
			for (int i = 0; i < (window_size - 2.f*pedestrian_radius)/bin_width; i++)
			{
				for (int j = 0; j < (corridor_width - 2.f*(pillar_radius + pedestrian_radius))/bin_width; j++)
				{
					Vec2 position = Vec2(-window_size/2.f + pedestrian_radius + 0.1f + i*bin_width,
							bin_width*j-corridor_width/2.f + pillar_radius + pedestrian_radius + 0.1f);
					if (position.norm() < no_go_radius)
						continue;
					circle_agents.push_back(RDS2CircleAgent(position - p_ref,
						0.f, RDS2CircleConfiguration(1.f,1.f,0.05f,1.6f, Circle(p_ref, pedestrian_radius), p_ref)));
				}
			}
			// static agents (walls)
			for (int i = 0; i < int(window_size/2.f/pillar_radius + 1); i++)
			{
				for (int j = -1; j < 2; j += 2)
				{
					circle_agents.push_back(RDS2CircleAgent(
						Vec2(-window_size/2.f + i*pillar_radius*2.f, -corridor_width/2.f*float(j)) - p_ref,
						0.f, RDS2CircleConfiguration(1.f,1.f,0.05f, 0.f, Circle(p_ref, pillar_radius), p_ref)));
				}
			}
			// static agents (obstacle)
			//circle_agents.push_back(RDS2CircleAgent(
			//	Vec2(window_size/2.f*0.75, 0.f) - p_ref,
			//	0.f, RDS2CircleConfiguration(1.f,1.f,0.05f, 0.f, Circle(p_ref, 5.f), p_ref)));

			generate_all_v_preferred();
			simulate("Hierarchical RVO-RDS control", window_size, simu_index, corridor_width);

			std::cout << "Average speed around robot for Hierarchical RVO-RDS: " << average_speed_around_robot_hierarchical << std::endl;
			std::cout << "Average speed around robot for RVO only: " << average_speed_around_robot_rvo_only << std::endl;
			std::cout << "Average forward velocity around robot for Hierarchical RVO-RDS: " << average_forward_velocity_around_robot_hierarchical << std::endl;
			std::cout << "Average forward velocity around robot for RVO only: " << average_forward_velocity_around_robot_rvo_only << std::endl;

			//if (argc > 2)
				break;
		}
	}
}