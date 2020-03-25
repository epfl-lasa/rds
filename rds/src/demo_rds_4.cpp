#include "rds_4_agent.hpp"
#include "rvo_3_agent.hpp"
#include "rds_4.hpp"
#include "geometry.hpp"
#include "capsule.hpp"
#include "gui.hpp"

#include <RVO.h> // external official RVO2 library

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <chrono>
#include <thread>
#include <random>
#include <iostream>
#include <iomanip>
#include <fstream>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::Capsule;

RDS4CapsuleAgent robot;
std::vector<Capsule> capsule_robot_global;
MovingCapsule moving_capsule_v_preferred_robot_global;
std::vector<RVO3Agent> rvo_3_agents;
std::vector<Circle> circles_rvo_3_agents_global;
std::vector<MovingCircle> moving_circles_rvo_3_agents_global;
std::vector<MovingCircle> moving_circles_v_preferred_rvo_3_agents_global;

std::vector<RVO::Vector2> all_v_preferred;

std::vector<Circle> all_objects_global_rvo_only;

std::default_random_engine generator(1);

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

float collision_delta;
unsigned long int rvo_only_collision_count = 0;
unsigned long int rds_4_rvo_3_collision_count = 0;

unsigned long int n_steps = 0;
const float speed_measurement_radius = 5.f;
double average_speed_around_robot_rds = 0.f;
double average_speed_around_robot_rvo_only = 0.f;

double robot_path_length_rds = 0.0;
double robot_path_length_rvo_only = 0.0;
double robot_horizontal_progress_rds = 0.0;
double robot_horizontal_progress_rvo_only = 0.0;
Vec2 robot_position_backup_rds(-666.f, -666.f);
Vec2 robot_position_backup_rvo_only(-666.f, -666.f);

double closest_distance_avrg_rds = 0.0;
double closest_distance_avrg_rvo_only = 0.0;

Vec2 normalized(const RVO::Vector2& v)
{
	return (Vec2(v.x(), v.y())).normalized();
}

Vec2 toVec2(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

float separation(const Circle& c1, const Circle& c2)
{
	return (c1.center - c2.center).norm() - c1.radius - c2.radius;
}

bool check_collision(const Circle& c1, const Circle& c2)
{
	if (separation(c1, c2) > collision_delta)
		return false;
	else
		return true;
}

void count_collisions()
{
	for (std::vector<Circle>::size_type i = 0; i != all_objects_global_rvo_only.size() - 1; i++)
	{
		for (std::vector<Circle>::size_type j = i + 1; j != all_objects_global_rvo_only.size(); j++)
		{
			if ((i != 0) && (rvo_3_agents[i-1].rvo_configuration.v_max == 0.f) &&
				(rvo_3_agents[j-1].rvo_configuration.v_max == 0.f))
				continue;
			if (check_collision(all_objects_global_rvo_only[i], all_objects_global_rvo_only[j]))
				rvo_only_collision_count++;
		}
	}
	for (std::vector<Circle>::size_type i = 0; i != circles_rvo_3_agents_global.size() - 1; i++)
	{
		for (std::vector<Circle>::size_type j = i + 1; j != circles_rvo_3_agents_global.size(); j++)
		{
			if ((rvo_3_agents[i].rvo_configuration.v_max == 0.f) &&
				(rvo_3_agents[j].rvo_configuration.v_max == 0.f))
				continue;
			if (check_collision(circles_rvo_3_agents_global[i], circles_rvo_3_agents_global[j]))
				rds_4_rvo_3_collision_count++;
		}
	}
	for (std::vector<Circle>::size_type i = 0; i != circles_rvo_3_agents_global.size(); i++)
	{
		Vec2 pt_segment;
		capsule_robot_global[0].closestMidLineSegmentPoint(circles_rvo_3_agents_global[i].center, &pt_segment);
		if (check_collision(circles_rvo_3_agents_global[i], Circle(pt_segment, capsule_robot_global[0].radius())))
			rds_4_rvo_3_collision_count++;
	}
}

void update_speed_measurements(RVO::RVOSimulator* sim_rvo_only)
{
	unsigned int n_agents_rds = 0;
	double speed_avrg_now_rds = 0.f;
	for (std::vector<RVO3Agent>::size_type i = 0; i != rvo_3_agents.size(); i++)
	{
		if ((rvo_3_agents[i].position - robot.position).norm() > speed_measurement_radius)
			continue;
		speed_avrg_now_rds = (n_agents_rds*speed_avrg_now_rds +
			rvo_3_agents[i].last_step_velocity.norm())/double(n_agents_rds + 1);
		n_agents_rds++;
	}
	average_speed_around_robot_rds = (n_steps*average_speed_around_robot_rds + speed_avrg_now_rds)/double(n_steps + 1);

	Vec2 p_robot(sim_rvo_only->getAgentPosition(0).x(), sim_rvo_only->getAgentPosition(0).y());
	unsigned int n_agents_rvo_only = 0;
	double speed_avrg_now_rvo_only = 0.f;
	for (unsigned long int i = 1; i != sim_rvo_only->getNumAgents(); i++)
	{
		Vec2 p_global(toVec2(sim_rvo_only->getAgentPosition(i)));
		if (((p_global - p_robot).norm()  < speed_measurement_radius))// && (sim_rvo_only->getAgentMaxSpeed(i) != 0.f))
		{
			double speed = (toVec2(sim_rvo_only->getAgentVelocity(i))).norm();
			speed_avrg_now_rvo_only = (n_agents_rvo_only*speed_avrg_now_rvo_only + speed)/double(n_agents_rvo_only + 1);
			n_agents_rvo_only++;
		}
	}
	average_speed_around_robot_rvo_only = 
		(n_steps*average_speed_around_robot_rvo_only + speed_avrg_now_rvo_only)/double(n_steps + 1);
	n_steps++;
}

void update_distance_measurements(RVO::RVOSimulator* sim_rvo_only)
{
	if (robot_position_backup_rds.x == -666.f)
	{
		robot_position_backup_rds = capsule_robot_global[0].center_a();
		robot_position_backup_rvo_only = all_objects_global_rvo_only[0].center;
		return;
	}

	robot_horizontal_progress_rds += robot.position.x - robot_position_backup_rds.x;
	robot_horizontal_progress_rvo_only += (toVec2(sim_rvo_only->getAgentPosition(0))).x - robot_position_backup_rvo_only.x;
	robot_path_length_rds += (robot.position - robot_position_backup_rds).norm();
	robot_path_length_rvo_only += (toVec2(sim_rvo_only->getAgentPosition(0)) - robot_position_backup_rvo_only).norm();
	robot_position_backup_rds = robot.position;
	robot_position_backup_rvo_only = toVec2(sim_rvo_only->getAgentPosition(0));

	float closest_distance_now_rvo_only = 10000.f;
	for (std::vector<Circle>::size_type j = 1; j != all_objects_global_rvo_only.size(); j++)
	{
		if (separation(all_objects_global_rvo_only[0], all_objects_global_rvo_only[j]) < closest_distance_now_rvo_only)
			closest_distance_now_rvo_only = separation(all_objects_global_rvo_only[0], all_objects_global_rvo_only[j]);
	}
	closest_distance_avrg_rvo_only = (n_steps*closest_distance_avrg_rvo_only + closest_distance_now_rvo_only)/double(n_steps + 1);

	float closest_distance_now_rds = 10000.f;
	for (std::vector<Circle>::size_type i = 0; i != circles_rvo_3_agents_global.size(); i++)
	{
		Vec2 pt_segment;
		capsule_robot_global[0].closestMidLineSegmentPoint(circles_rvo_3_agents_global[i].center, &pt_segment);
		if (separation(circles_rvo_3_agents_global[i], Circle(pt_segment, capsule_robot_global[0].radius())) < closest_distance_now_rds)
			closest_distance_now_rds = separation(circles_rvo_3_agents_global[i], Circle(pt_segment, capsule_robot_global[0].radius()));
	}
	closest_distance_avrg_rds = (n_steps*closest_distance_avrg_rds + closest_distance_now_rds)/double(n_steps + 1);
}

void generate_all_v_preferred(float v_robot, float v_max_robot, float v_crowd, float v_max_crowd)
{
	RVO::Vector2 v_preferred_robot(std::min(v_max_robot, std::max(0.2f*v_max_robot, v_robot)), 0.f);
	all_v_preferred.push_back(v_preferred_robot);

	std::normal_distribution<float> normal_distribution (0.0,0.5);
	for (auto& a : rvo_3_agents)
	{	
		RVO::Vector2 v_preferred(std::min(v_max_crowd, std::max(0.2f*v_max_crowd, 
			v_crowd + float(normal_distribution(generator)))), 0.f);
		all_v_preferred.push_back(v_preferred);
	}
}

bool update_gui(GUI* gui, GUI* gui_rvo_only, bool quick_measurement_mode = false)
{
	count_collisions();

	if (!quick_measurement_mode)
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
	}
	
	bool still_open = (gui->update() == 0) | (gui_rvo_only->update() == 0);
	return still_open;
}

void update_objects_for_gui_and_with_v_preferred(const RVO::RVOSimulator& rvo_simulator)
{
	Vec2 v_result;
	robot.transformVectorLocalToGlobal(robot.rds_configuration.robot_shape.center_a(), &v_result);
	Vec2 center_a(robot.position + v_result);
	robot.transformVectorLocalToGlobal(robot.rds_configuration.robot_shape.center_b(), &v_result);
	Vec2 center_b(robot.position + v_result);
	capsule_robot_global[0] = Capsule(capsule_robot_global[0].radius(), center_a, center_b);

	moving_capsule_v_preferred_robot_global.capsule = capsule_robot_global[0];
	robot.transformVectorLocalToGlobal(robot.rds_configuration.p_ref, &v_result);
	moving_capsule_v_preferred_robot_global.p_ref = robot.position + v_result;
	robot.transformVectorGlobalToLocal(moving_capsule_v_preferred_robot_global.velocity_p_ref, &v_result);
	moving_capsule_v_preferred_robot_global.omega = -v_result.x/robot.rds_configuration.p_ref.y;

	for (std::vector<Circle>::size_type i = 0; i != circles_rvo_3_agents_global.size(); i++)
	{
		circles_rvo_3_agents_global[i].center = rvo_3_agents[i].position;
		moving_circles_v_preferred_rvo_3_agents_global[i].circle.center = rvo_3_agents[i].position;
	}

	for (std::vector<Circle>::size_type i = 0; i != all_objects_global_rvo_only.size(); i++)
	{
		all_objects_global_rvo_only[i].center = Vec2(rvo_simulator.getAgentPosition(i).x(),
			rvo_simulator.getAgentPosition(i).y());
	}
}

void update_moving_objects_rvo_3_agents()
{
	for (std::vector<RVO3Agent>::size_type i = 0; i != rvo_3_agents.size(); i++)
	{
		moving_circles_rvo_3_agents_global[i].circle.center = rvo_3_agents[i].position;
		moving_circles_rvo_3_agents_global[i].velocity = rvo_3_agents[i].last_step_velocity;
	}
}

void simulate(float screen_size, float corridor_width, bool quick_measurement_mode = false)
{
	GUI gui("RDS-4 (robot) and RVO-3 (pedestrians)", screen_size);
	gui.circles = &circles_rvo_3_agents_global;
	gui.capsules = &capsule_robot_global;

	circles_rvo_3_agents_global.resize(rvo_3_agents.size());
	for (std::vector<Circle>::size_type i = 0; i != circles_rvo_3_agents_global.size(); i++)
		circles_rvo_3_agents_global[i].radius = rvo_3_agents[i].rvo_configuration.radius;
	capsule_robot_global.resize(1);
	capsule_robot_global[0] = robot.rds_configuration.robot_shape;

	GUI gui_rvo_only("Official RVO-2 (robot and pedestrians)", screen_size);
	gui_rvo_only.circles = &all_objects_global_rvo_only;

	all_objects_global_rvo_only.resize(rvo_3_agents.size() + 1);
	all_objects_global_rvo_only[0].radius = robot.rds_configuration.robot_shape.radius() + std::max(
		(robot.rds_configuration.robot_shape.center_a() - robot.rds_configuration.p_ref).norm(),
		(robot.rds_configuration.robot_shape.center_b() - robot.rds_configuration.p_ref).norm());

	for (std::vector<Circle>::size_type i = 1; i != all_objects_global_rvo_only.size(); i++)
		all_objects_global_rvo_only[i].radius = rvo_3_agents[i - 1].rvo_configuration.radius;

	GuiColor green;
	green.g = 255;
	green.r = green.b = 0;
	gui_rvo_only.circles_colors.push_back(green);

	GuiColor yellow, red, white;
	red.r = 255;
	red.g = red.b = 0;
	yellow.r = yellow.g = 255;
	yellow.b = 0;
	white.r = white.g = white.b = 255;
	for (unsigned long int i = 1; i != all_v_preferred.size(); i++)
	{
		if (rvo_3_agents[i-1].rvo_configuration.v_max == 0.f)
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

	moving_circles_v_preferred_rvo_3_agents_global.resize(rvo_3_agents.size());
	moving_circles_rvo_3_agents_global.resize(rvo_3_agents.size());
	for (std::vector<RVO3Agent>::size_type i = 0; i != rvo_3_agents.size(); i++)
	{
		moving_circles_v_preferred_rvo_3_agents_global[i].velocity = toVec2(all_v_preferred[i + 1]);
		moving_circles_v_preferred_rvo_3_agents_global[i].circle.radius = circles_rvo_3_agents_global[i].radius;
		moving_circles_rvo_3_agents_global[i].circle.radius = circles_rvo_3_agents_global[i].radius;
	}
	moving_capsule_v_preferred_robot_global.velocity_p_ref = toVec2(all_v_preferred[0]);

	RVO::RVOSimulator sim_rvo_only;
	Vec2 v_result;
	robot.transformVectorLocalToGlobal(robot.rds_configuration.p_ref, &v_result);
	Vec2 robot_center_position = robot.position + v_result;
	sim_rvo_only.addAgent(RVO::Vector2(robot_center_position.x, robot_center_position.y), 15.0f, 10,
		robot.rds_configuration.tau, robot.rds_configuration.tau,
			all_objects_global_rvo_only[0].radius + robot.rds_configuration.delta, robot.rds_configuration.v_max);
	for (auto& a : rvo_3_agents)
	{
		sim_rvo_only.addAgent(RVO::Vector2(a.position.x, a.position.y), 15.0f, 10, a.rvo_configuration.tau,
			a.rvo_configuration.tau, a.rvo_configuration.radius + a.rvo_configuration.delta, a.rvo_configuration.v_max);
	}
	for (unsigned long int i = 0; i != sim_rvo_only.getNumAgents(); i++)
		sim_rvo_only.setAgentPrefVelocity(i, all_v_preferred[i]);

	float dt = gui_cycle_time.count()*0.001f;
	sim_rvo_only.setTimeStep(dt);

	do
	{
		update_objects_for_gui_and_with_v_preferred(sim_rvo_only);
		for (std::vector<RVO3Agent>::size_type i = 0; i != rvo_3_agents.size(); i++)
		{
			rvo_3_agents[i].stepEuler(dt, toVec2(all_v_preferred[i + 1]),
				moving_circles_v_preferred_rvo_3_agents_global, moving_capsule_v_preferred_robot_global, i);
		}
		update_moving_objects_rvo_3_agents();
		robot.stepEuler(dt, toVec2(all_v_preferred[0]), moving_circles_rvo_3_agents_global);

		sim_rvo_only.doStep();
		update_speed_measurements(&sim_rvo_only);
		update_distance_measurements(&sim_rvo_only);
	}
	while (update_gui(&gui, &gui_rvo_only, quick_measurement_mode) && (n_steps < (unsigned long int)(40.0/dt)));
}

int main(int argc, char** argv)
{
	double v_robot, v_crowd, density;
	if (argc > 3)
	{
		v_robot = std::stod(argv[1]);
		v_crowd = std::stod(argv[2]);
		density = std::stod(argv[3]);
	}
	else
	{
		std::cout << "Provide 3 arguments to define the robot speed, the crowd speed, and the density, respectively." << std::endl;
		return 0;
	}

	float delta = 0.05f;
	float tau = 1.f/2.f/density;
	collision_delta = 0.f;
	float v_max_robot = 1.8f;
	float capsule_radius = 0.25f;
	float reference_point_y = 0.2f;
	float capsule_segment_length = 0.5f;
	robot = RDS4CapsuleAgent(Vec2(0.f, 0.f), -M_PI/2.f, RDSCapsuleConfiguration(tau, delta, v_max_robot,
		Capsule(capsule_radius, Vec2(0.f, reference_point_y), Vec2(0.f, reference_point_y-capsule_segment_length)),
		Vec2(0.f, reference_point_y)));

	float window_size = 50.f;
	float corridor_width = 20.f;
	float pillar_radius = 0.5f;
	float pedestrian_radius = 0.25f;
	float bin_width = std::sqrt(1.f/density);

	float v_max_crowd = 1.8f;
	// moving agents
	float no_go_radius = capsule_segment_length + capsule_radius + 0.5;
	std::uniform_real_distribution<float> uniform_distribution (-0.05,0.05);
	for (int i = 0; i < (window_size - 2.f*pedestrian_radius)/bin_width; i++)
	{
		for (int j = 0; j < (corridor_width - 2.f*(pillar_radius + pedestrian_radius))/bin_width; j++)
		{
			Vec2 position = Vec2(-window_size/2.f + pedestrian_radius + 0.1f + i*bin_width + uniform_distribution(generator),
					bin_width*j-corridor_width/2.f + pillar_radius + pedestrian_radius + 0.1f + uniform_distribution(generator));
			if (position.norm() < no_go_radius)
				continue;
			rvo_3_agents.push_back(RVO3Agent(position,
				RVO3Configuration(tau, delta, v_max_crowd, pedestrian_radius)));
		}
	}
	// static agents (walls)
	for (int i = 0; i < int(window_size/2.f/pillar_radius + 1); i++)
	{
		for (int j = -1; j < 2; j += 2)
		{
			rvo_3_agents.push_back(RVO3Agent(
				Vec2(-window_size/2.f + i*pillar_radius*2.f, -corridor_width/2.f*float(j)),
				RVO3Configuration(tau, delta, 0.f, pillar_radius)));
		}
	}

	generate_all_v_preferred(v_robot, v_max_robot, v_crowd, v_max_crowd);
	simulate(window_size, corridor_width, false);

	std::cout << "Collision counts:" << std::endl;
	std::cout << "RDS-4 with RVO-3: " << rds_4_rvo_3_collision_count << std::endl;
	std::cout << "RVO-2 (official): " << rvo_only_collision_count << std::endl;

	std::cout << "Agents speed around the robot (average over time):" << std::endl;
	std::cout << "RDS-4 with RVO-3: " << average_speed_around_robot_rds << std::endl;
	std::cout << "RVO-2 (official): " << average_speed_around_robot_rvo_only << std::endl;

	std::cout << "Robot path length:" << std::endl;
	std::cout << "RDS-4 with RVO-3: " << robot_path_length_rds << std::endl;
	std::cout << "RVO-2 (official): " << robot_path_length_rvo_only << std::endl;

	std::cout << "Robot horizontal progress:" << std::endl;
	std::cout << "RDS-4 with RVO-3: " << robot_horizontal_progress_rds << std::endl;
	std::cout << "RVO-2 (official): " << robot_horizontal_progress_rvo_only << std::endl;

	std::cout << "Robot-objects distance minimum (average over time):" << std::endl;
	std::cout << "RDS-4 with RVO-3: " << closest_distance_avrg_rds << std::endl;
	std::cout << "RVO-2 (official): " << closest_distance_avrg_rvo_only << std::endl;

	return 0;
  std::string filename = "rds_4_table.txt";
  std::fstream s(filename, s.out | s.app);
  if (!s.is_open()) {
    std::cout << "failed to open " << filename << '\n';
  } else {
    // write
    s << std::setw(4) << rds_4_rvo_3_collision_count << ",  ";
    s << std::setw(4) << rvo_only_collision_count << ",    ";
    s << std::setw(8) << average_speed_around_robot_rds << ",  ";
    s << std::setw(8) << average_speed_around_robot_rvo_only << ",    ";
    s << std::setw(8) << robot_horizontal_progress_rds << ",  ";
    s << std::setw(8) << robot_horizontal_progress_rvo_only << ",    ";
    s << std::setw(8) << closest_distance_avrg_rds << ",  ";
    s << std::setw(8) << closest_distance_avrg_rvo_only << std::endl;
  }
  s.close();
}