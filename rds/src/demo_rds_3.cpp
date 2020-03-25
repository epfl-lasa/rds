#include "geometry.hpp"
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;

#include "capsule.hpp"
using Geometry2D::Capsule;

#include "gui.hpp"
#include "rds_3_agent.hpp"

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

std::vector<RDS3CircleAgent> circle_agents;
std::vector<Circle> circle_objects_global;
std::vector<RDS3CapsuleAgent> capsule_agents;
std::vector<Capsule> capsule_objects_global;

std::vector<Circle> circle_objects_global_rvo_only;

std::vector<RVO::Vector2> all_v_preferred;

std::default_random_engine generator(1);

unsigned long int n_steps = 0;
const float speed_measurement_radius = 5.f;
double average_speed_around_robot_hierarchical = 0.f;
double average_speed_around_robot_rvo_only = 0.f;
double average_forward_velocity_around_robot_hierarchical = 0.f;
double average_forward_velocity_around_robot_rvo_only = 0.f;

std::vector<double> v_average_speed_around_robot_hierarchical;
std::vector<double> v_average_speed_around_robot_rvo_only;
std::vector<double> v_average_forward_velocity_around_robot_hierarchical;
std::vector<double> v_average_forward_velocity_around_robot_rvo_only;

Vec2 normalized(const RVO::Vector2& v)
{
	return (Vec2(v.x(), v.y())).normalized();
}

Vec2 toVec2(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

bool update_gui(GUI* gui, GUI* gui_rvo_only, bool quick_measurement_mode = false)
{
	if (!quick_measurement_mode)
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
	}
	
	for (auto& cap : capsule_agents) // add the robot circle (serves for display only)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_3_configuration.p_ref, &v_result);
		Vec2 p_ref_global(cap.position + v_result);
		circle_objects_global.push_back(Circle(p_ref_global, cap.rds_3_configuration.robot_shape.radius()));
	}
	bool still_open = (gui->update() == 0) | (gui_rvo_only->update() == 0);
	for (auto& cap : capsule_agents)
		circle_objects_global.pop_back(); // remove again the robot circle (serves for display only)
	return still_open;
}

void generate_all_v_preferred(float v_robot, float v_max_robot, float v_crowd, float v_max_crowd)
{
	std::normal_distribution<float> normal_distribution (0.0,0.5);
	for (auto& cap : capsule_agents)
	{
		RVO::Vector2 v_preferred(std::min(v_max_robot, std::max(0.2f*v_max_robot, 
			v_robot)), 0.f);
		all_v_preferred.push_back(v_preferred);
	}
	for (auto& cir : circle_agents)
	{	
		RVO::Vector2 v_preferred(std::min(v_max_crowd, std::max(0.2f*v_max_crowd, 
			v_crowd + float(normal_distribution(generator)))), 0.f);
		all_v_preferred.push_back(v_preferred);
	}
}

void update_objects_global()
{
	for (std::vector<Circle>::size_type i = 0; i != circle_objects_global.size(); i++)
	{
		Vec2 v_result;
		circle_agents[i].transformVectorLocalToGlobal(circle_agents[i].rds_3_configuration.robot_shape.center, &v_result);
		circle_objects_global[i].center = circle_agents[i].position + v_result;
	}
	for (std::vector<Capsule>::size_type i = 0; i != capsule_objects_global.size(); i++)
	{
		Vec2 v_result;
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_3_configuration.robot_shape.center_a(), &v_result);
		Vec2 center_a(capsule_agents[i].position + v_result);
		capsule_agents[i].transformVectorLocalToGlobal(capsule_agents[i].rds_3_configuration.robot_shape.center_b(), &v_result);
		Vec2 center_b(capsule_agents[i].position + v_result);
		capsule_objects_global[i] = Capsule(capsule_objects_global[i].radius(), center_a, center_b);
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

void shift_all_agents(RVO::RVOSimulator* sim_rvo_only)
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

void reinsert_agents(float window_size, float corridor_width, RVO::RVOSimulator* sim_rvo_only)
{
	std::uniform_real_distribution<float> uniform_distribution (-1.0,1.0);
	float offset = 1.f;
	for (auto& cir : circle_agents)
	{
		if (cir.position.x < -window_size/2.f - offset)
		{
			cir.position.x = window_size/2.f;
			if (cir.rds_3_configuration.v_max != 0.f)
				cir.position.y = (corridor_width -2.f)/2.f*float(uniform_distribution(generator));
		}
		else if (cir.position.x > window_size/2.f + offset)
		{
			cir.position.x = -window_size/2.f;
			if (cir.rds_3_configuration.v_max != 0.f)
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

void print_speed_measurements()
{
	std::cout << "Average speed around robot for Hierarchical RVO-RDS: " << average_speed_around_robot_hierarchical << std::endl;
	std::cout << "Average speed around robot for RVO only: " << average_speed_around_robot_rvo_only << std::endl;
	std::cout << "Average forward velocity around robot for Hierarchical RVO-RDS: " << average_forward_velocity_around_robot_hierarchical << std::endl;
	std::cout << "Average forward velocity around robot for RVO only: " << average_forward_velocity_around_robot_rvo_only << std::endl;
	v_average_speed_around_robot_hierarchical.push_back(average_speed_around_robot_hierarchical);
	v_average_speed_around_robot_rvo_only.push_back(average_speed_around_robot_rvo_only);
	v_average_forward_velocity_around_robot_hierarchical.push_back(average_forward_velocity_around_robot_hierarchical);
	v_average_forward_velocity_around_robot_rvo_only.push_back(average_forward_velocity_around_robot_rvo_only);
}

void plot_speed_measurements()
{
	GUI plot("Average forward velocity around robot (red=rvo, white=rvo-rds)", 2.f);
	std::vector<Vec2> points;
	plot.points = &points;
	GuiColor red;
	red.r = 255;
	red.g = red.b = 0;
	double x = 0.0;
	for (auto& m : v_average_forward_velocity_around_robot_rvo_only)
	{
		points.push_back(Vec2(x - 1.f, m - 1.f));
		plot.points_colors.push_back(red);
		x += 2.0/v_average_forward_velocity_around_robot_rvo_only.size();
	}
	x = 0.0;
	for (auto& m : v_average_forward_velocity_around_robot_hierarchical)
	{
		points.push_back(Vec2(x - 1.f, m - 1.f));
		x += 2.0/v_average_forward_velocity_around_robot_rvo_only.size();
	}
	plot.blockingShowUntilClosed();
}

void update_speed_measurements(RVO::RVOSimulator* sim_rvo_only)
{
	Vec2 v_result;
	capsule_agents[0].transformVectorLocalToGlobal(capsule_agents[0].rds_3_configuration.p_ref, &v_result);
	Vec2 p_ref_robot = capsule_agents[0].position + v_result;

	unsigned int n_agents_hierarchical = 0;
	double speed_avrg_now_hierarchical = 0.f;
	double v_f_avrg_now_hier = 0.f;
	unsigned long int sim_i = 0;
	for (auto& cap : capsule_agents)
	{
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(cap.rds_3_configuration.p_ref, &v_result);
		Vec2 p_ref_global = cap.position + v_result;
		if (((p_ref_robot - p_ref_global).norm() < speed_measurement_radius) && (cap.rds_3_configuration.v_max != 0.f))
		{
			speed_avrg_now_hierarchical =
				(n_agents_hierarchical*speed_avrg_now_hierarchical + cap.last_step_p_ref_velocity.norm())/double(n_agents_hierarchical + 1);
			double v_forward = cap.last_step_p_ref_velocity.dot(normalized(all_v_preferred[sim_i]));
			v_f_avrg_now_hier = (n_agents_hierarchical*v_f_avrg_now_hier + v_forward)/double(n_agents_hierarchical + 1);
			n_agents_hierarchical++;
		}
		sim_i++;
	}
	for (auto& cir : circle_agents)
	{
		Vec2 v_result;
		cir.transformVectorLocalToGlobal(cir.rds_3_configuration.p_ref, &v_result);
		Vec2 p_ref_global = cir.position + v_result;
		if (((p_ref_robot - p_ref_global).norm() < speed_measurement_radius) && (cir.rds_3_configuration.v_max != 0.f))
		{
			speed_avrg_now_hierarchical =
				(n_agents_hierarchical*speed_avrg_now_hierarchical + cir.last_step_p_ref_velocity.norm())/double(n_agents_hierarchical + 1);
			double v_forward = cir.last_step_p_ref_velocity.dot(normalized(all_v_preferred[sim_i]));
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
	double speed_avrg_now_rvo_only = 0.f;
	double v_f_avrg_now_rvoo = 0.f;
	for (unsigned long int i = 0; i != sim_rvo_only->getNumAgents(); i++)
	{
		Vec2 p_global(sim_rvo_only->getAgentPosition(i).x(), sim_rvo_only->getAgentPosition(i).y());
		if (((p_global - p_robot).norm()  < speed_measurement_radius) && (sim_rvo_only->getAgentMaxSpeed(i) != 0.f))
		{
			double speed = (Vec2(sim_rvo_only->getAgentVelocity(i).x(), sim_rvo_only->getAgentVelocity(i).y())).norm();
			speed_avrg_now_rvo_only = (n_agents_rvo_only*speed_avrg_now_rvo_only + speed)/double(n_agents_rvo_only + 1);
			double v_forward = (toVec2(sim_rvo_only->getAgentVelocity(i))).dot(normalized(all_v_preferred[i]));
			v_f_avrg_now_rvoo = (n_agents_rvo_only*v_f_avrg_now_rvoo + v_forward)/double(n_agents_rvo_only + 1);
			n_agents_rvo_only++;
		}
	}
	average_speed_around_robot_rvo_only = 
		(n_steps*average_speed_around_robot_rvo_only + speed_avrg_now_rvo_only)/double(n_steps + 1);
	average_forward_velocity_around_robot_rvo_only = (n_steps*average_forward_velocity_around_robot_rvo_only +
		v_f_avrg_now_rvoo)/double(n_steps + 1);
	n_steps++;
	if (n_steps % 200 == 0)
		print_speed_measurements();
}

void simulate(const char* title, float screen_size, int simu_index, float corridor_width = 0,
	bool quick_measurement_mode = false)
{
	GUI gui(title, screen_size);
	gui.circles = &circle_objects_global;
	gui.capsules = &capsule_objects_global;

	{
		circle_objects_global.resize(circle_agents.size());
		for (std::vector<Circle>::size_type i = 0; i != circle_objects_global.size(); i++)
			circle_objects_global[i].radius = circle_agents[i].rds_3_configuration.robot_shape.radius;
		capsule_objects_global.resize(capsule_agents.size());
		for (std::vector<Capsule>::size_type i = 0; i != capsule_objects_global.size(); i++)
			capsule_objects_global[i] = Capsule(capsule_agents[i].rds_3_configuration.robot_shape.radius(), Vec2(0.f, -1.f), Vec2(0.f, 1.f));
	}

	GUI gui_rvo_only((std::string("RVO only: ") + std::string(title)).c_str(), screen_size);
	gui_rvo_only.circles = &circle_objects_global_rvo_only;
	GuiColor green;
	green.g = 255;
	green.r = green.b = 0;
	gui_rvo_only.circles_colors.push_back(green);

	if (all_v_preferred.size() != 0)
	{
		GuiColor yellow, red, white;
		red.r = 255;
		red.g = red.b = 0;
		yellow.r = yellow.g = 255;
		yellow.b = 0;
		white.r = white.g = white.b = 255;
		for (unsigned long int i = 1; i != all_v_preferred.size(); i++)
		{
			if (circle_agents[i-1].rds_3_configuration.v_max == 0.f)
			{
				gui_rvo_only.circles_colors.push_back(white);
				gui.circles_colors.push_back(white);
			}
			else if (all_v_preferred[i].x() < 0.f)
			{
				gui_rvo_only.circles_colors.push_back(red);
				gui.circles_colors.push_back(red);
			}
			else
			{
				gui_rvo_only.circles_colors.push_back(yellow);
				gui.circles_colors.push_back(yellow);
			}
		}
	}

	update_objects_global();
	RVO::RVOSimulator sim_rvo_only;
	for (auto& cap : capsule_agents)
	{
		const Capsule& c(cap.rds_3_configuration.robot_shape);
		Vec2 circle_center_local = (c.center_a() + c.center_b())/2.f;
		Vec2 v_result;
		cap.transformVectorLocalToGlobal(circle_center_local, &v_result);
		float r = (c.center_a() - c.center_b()).norm()/2.f + c.radius() + cap.rds_3_configuration.delta;
		sim_rvo_only.addAgent(RVO::Vector2((cap.position + v_result).x, (cap.position + v_result).y), 15.0f, 10, 5.0f, 5.0f,
			r, cap.rds_3_configuration.v_max);
		circle_objects_global_rvo_only.push_back(Circle(Vec2(0.f, 0.f), r));
	}
	for (unsigned long int i = 0; i < circle_agents.size(); i++)
	{
		float r = circle_agents[i].rds_3_configuration.robot_shape.radius +
			circle_agents[i].rds_3_configuration.delta;
		sim_rvo_only.addAgent(RVO::Vector2(circle_objects_global[i].center.x, circle_objects_global[i].center.y),
			15.0f, 10, 5.0f, 5.0f, r, circle_agents[i].rds_3_configuration.v_max);
		circle_objects_global_rvo_only.push_back(Circle(Vec2(0.f, 0.f), r));
	}

	int n_iterations = 5;
	float dt = gui_cycle_time.count()*0.001f/n_iterations;
	sim_rvo_only.setTimeStep(dt*n_iterations);
	unsigned long int n_simulation_steps = 0;
	unsigned long int n_start_measurement(60.0/dt/n_iterations);
	do
	{
		for (int k = 0; k != n_iterations; k++)
		{
			update_objects_global();

			unsigned long int i_sim = 0;
			for (std::vector<RDS3CapsuleAgent>::size_type i = 0; i != capsule_agents.size(); i++)
			{
				RVO::Vector2 v_nominal = all_v_preferred[i_sim];
				capsule_agents[i].stepEuler(dt, Vec2(v_nominal.x(), v_nominal.y()), circle_objects_global);
				i_sim++;
			}
			for (std::vector<RDS3CircleAgent>::size_type i = 0; i != circle_agents.size(); i++)
			{
				RVO::Vector2 v_nominal = all_v_preferred[i_sim];
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

		if (n_simulation_steps > n_start_measurement)
			update_speed_measurements(&sim_rvo_only);
		n_simulation_steps++;

		if (simu_index == 2)
		{
			shift_all_agents(&sim_rvo_only);
			reinsert_agents(screen_size, corridor_width, &sim_rvo_only);
		}
	}
	while (update_gui(&gui, &gui_rvo_only, quick_measurement_mode));
}

int main(int argc, char** argv)
{
	double v_robot, v_crowd;
	if (argc > 2)
	{
		v_robot = std::stod(argv[1]);
		v_crowd = std::stod(argv[2]);
	}
	else
		std::cout << "Provide two arguments to define the robot and crowd speed respectively." << std::endl;

	float v_max_robot = 1.8f;
	float capsule_radius = 0.5f;
	float reference_point_y = 0.2f;
	float capsule_segment_length = 0.5f;
	capsule_agents.push_back(RDS3CapsuleAgent(Vec2(0.f, 0.f), -M_PI/2.f, RDS3CapsuleConfiguration(1.f,0.05f, v_max_robot,
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
	float v_max_crowd = 1.8f;
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
			circle_agents.push_back(RDS3CircleAgent(position - p_ref,
				0.f, RDS3CircleConfiguration(1.f,0.05f, v_max_crowd, Circle(p_ref, pedestrian_radius), p_ref)));
		}
	}
	// static agents (walls)
	for (int i = 0; i < int(window_size/2.f/pillar_radius + 1); i++)
	{
		for (int j = -1; j < 2; j += 2)
		{
			circle_agents.push_back(RDS3CircleAgent(
				Vec2(-window_size/2.f + i*pillar_radius*2.f, -corridor_width/2.f*float(j)) - p_ref,
				0.f, RDS3CircleConfiguration(1.f,0.05f, 0.f, Circle(p_ref, pillar_radius), p_ref)));
		}
	}
	// static agents (obstacle)
	//circle_agents.push_back(RDS3CircleAgent(
	//	Vec2(window_size/2.f*0.75, 0.f) - p_ref,
	//	0.f, RDS3CircleConfiguration(1.f,0.05f, 0.f, Circle(p_ref, 5.f), p_ref)));

	generate_all_v_preferred(v_robot, v_max_robot, v_crowd, v_max_crowd);
	simulate("Hierarchical RVO-RDS control", window_size, 2, corridor_width, true);

	print_speed_measurements();
	plot_speed_measurements();
}