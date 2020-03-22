#include "rds_4_agent.hpp"
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
#include <iostream>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::Capsule;

RDS4CapsuleAgent robot;
std::vector<Capsule> capsule_robot_global;
std::vector<Circle> circles_global;
std::vector<MovingCircle> moving_circles_global;

float delta = 0.05f;
float tau = 2.f;
float v_max_robot = 1.8f;
float capsule_radius = 0.5f;
float reference_point_y = 0.2f;
float capsule_segment_length = 0.5f;
float speed_nominal_robot = 1.2f;
Vec2 start_position_robot(0.f, 0.f);
float window_size = 20.f;

float robot_rvo_radius = 0.f;

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

Vec2 get_robot_nominal_velocity(float time, const Vec2& robot_position)
{
	Vec2 goal = start_position_robot + Vec2((time + 0.5f)*speed_nominal_robot, 0.f);
	return speed_nominal_robot/std::max((goal - robot_position).norm(), 1.f)*(goal - robot_position);
}

Vec2 get_moving_circles_velocity(std::vector<Circle>::size_type i, double time, const Vec2& position)
{
	if (i == 0)
	{
		return Vec2(0.f, 0.f);
	}
	else if (i == 1)
	{
		Vec2 goal(5.f, -1.0f);
		return 1.5f/std::max((goal - position).norm(), 1.f)*(goal - position);
	}
	else
		return Vec2(0.f, 0.f);
}

void init_moving_circles()
{
	moving_circles_global.resize(circles_global.size());
	for (std::vector<Circle>::size_type i = 0; i != circles_global.size(); i++)
		moving_circles_global[i].circle.radius = circles_global[i].radius;
}

void init_capsule_robot()
{
	capsule_robot_global.resize(1);
}

void update_moving_circles_state(double time)
{
	for (std::vector<Circle>::size_type i = 0; i != circles_global.size(); i++)
	{
		moving_circles_global[i].circle.center = circles_global[i].center;
		moving_circles_global[i].velocity = get_moving_circles_velocity(i, time, circles_global[i].center);
	}
}

void update_robot_capsule_for_gui()
{
	Vec2 v_result;
	robot.transformVectorLocalToGlobal(robot.rds_configuration.robot_shape.center_a, &v_result);
	Vec2 center_a(robot.position + v_result);
	robot.transformVectorLocalToGlobal(robot.rds_configuration.robot_shape.center_b, &v_result);
	Vec2 center_b(robot.position + v_result);
	capsule_robot_global[0] = Capsule(capsule_robot_global[0].radius, center_a, center_b);
}

bool update_gui(GUI* gui)
{
	std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - t_gui_update));
	t_gui_update = std::chrono::high_resolution_clock::now();

	if (robot_rvo_radius != 0.f)
	{
		Vec2 v_result;
		robot.transformVectorLocalToGlobal(robot.rds_configuration.p_ref, &v_result);
		Vec2 robot_center(robot.position + v_result);
		circles_global.push_back(Circle(robot_center, robot_rvo_radius));
	}
	bool still_open = (gui->update() == 0);
	if (robot_rvo_radius != 0.f)
		circles_global.pop_back();
	return still_open;
}

void robot_stepEulerRVO(float dt, const Vec2& v_nominal, const std::vector<MovingCircle>& moving_circles)
{
	RVO::RVOSimulator sim;
	sim.setTimeStep(dt);

	robot_rvo_radius = capsule_radius + std::max(
		(robot.rds_configuration.robot_shape.center_a - robot.rds_configuration.p_ref).norm(),
		(robot.rds_configuration.robot_shape.center_b - robot.rds_configuration.p_ref).norm());
	Vec2 v_result;
	robot.transformVectorLocalToGlobal(robot.rds_configuration.p_ref, &v_result);
	Vec2 robot_center(robot.position + v_result);
	sim.addAgent(RVO::Vector2(robot_center.x, robot_center.y), 15.f, 10.f, tau, tau, robot_rvo_radius + delta, v_max_robot);

	for (auto& c : circles_global)
		sim.addAgent(RVO::Vector2(c.center.x, c.center.y), 15.f, 10.f, tau, tau, c.radius, 10.f);

	sim.setAgentPrefVelocity(0, RVO::Vector2(v_nominal.x, v_nominal.y));
	for (std::vector<Circle>::size_type i = 1; i != circles_global.size(); i++)
		sim.setAgentPrefVelocity(i, RVO::Vector2(moving_circles[i].velocity.x, moving_circles[i].velocity.y));

	sim.doStep();
	robot.position = robot.position + dt*Vec2(sim.getAgentVelocity(0).x(), sim.getAgentVelocity(0).y());
}

void simulate(bool use_rvo_2)
{
	GUI gui("RDS-4 evaluation in sparse crowd conditions", window_size);
	gui.circles = &circles_global;
	gui.capsules = &capsule_robot_global;

	float dt = gui_cycle_time.count()*0.001f;
	double time = 0.0;
	do
	{
		update_moving_circles_state(time);
		if (!use_rvo_2)
			robot.stepEuler(dt, get_robot_nominal_velocity(time, robot.position), moving_circles_global);
		else
			robot_stepEulerRVO(dt, get_robot_nominal_velocity(time, robot.position), moving_circles_global);
		for (std::vector<Circle>::size_type i = 0; i != circles_global.size(); i++)
			circles_global[i].center = circles_global[i].center + dt*moving_circles_global[i].velocity;
		time += dt;

		update_robot_capsule_for_gui();
	}
	while (update_gui(&gui));
}

int main()
{
	robot = RDS4CapsuleAgent(start_position_robot, -M_PI/2.f,
		RDSCapsuleConfiguration(tau, delta, v_max_robot,
		Capsule(capsule_radius, Vec2(0.f, reference_point_y), Vec2(0.f, reference_point_y-capsule_segment_length)),
		Vec2(0.f, reference_point_y)));
	init_capsule_robot();

	circles_global.push_back(Circle(Vec2(5.f, 4.f), 3.f));
	circles_global.push_back(Circle(Vec2(5.f, -5.f), 0.25f));
	init_moving_circles();

	simulate(true);

	return 0;
}