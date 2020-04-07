#include "gui.hpp"
#include "rds_orca_simulator.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdio>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;
using Geometry2D::BoundingCircles;
using AdditionalPrimitives2D::Polygon;

const float dt = 0.05f;

double robot_mean_distance_to_target = 0.0;
double time_counter = 0.0;

void update_mean(float distance)
{
	double w_old = time_counter/(time_counter + dt);
	double w_new = dt/(time_counter + dt);
	robot_mean_distance_to_target = w_old*robot_mean_distance_to_target + w_new*distance;
	time_counter += dt;
}

struct GuiWrap
{
	GuiWrap(const CrowdRdsOrcaSimulator& sim, float track_from_time)
	: m_gui("RDS-ORCA Simulator", 20.f, 700)
	, m_bounding_circles(sim.getBoundingCirclesRobot())
	, m_track_from_time(track_from_time)
	{
		m_gui.points = &m_points;
		m_gui.circles = &m_circles;
		m_gui.capsules = &m_capsules;
		m_gui.polygons = &m_polygons;
		for (auto& p : sim.getPedestrians())
		{
			m_circles.push_back(p.circle);
			Polygon line = {p.circle.center, p.circle.center};
			m_polygons.push_back(line);
		}
		m_capsules.push_back(sim.getRobot().rds_configuration.robot_shape);
		for (auto& c : m_bounding_circles.circles())
			m_circles.push_back(c);
		Polygon robot_line = {Vec2(), Vec2()};
		m_polygons.push_back(robot_line);
		if (sim.m_orca_orca)
			m_circles.push_back(sim.getOrcaOrcaCircle());
	}

	bool update(const CrowdRdsOrcaSimulator& sim)
	{
		int offset = 0;
		if (sim.m_orca_orca)
			offset = 1;
		for (std::vector<Circle>::size_type i = 0; i < m_circles.size() - m_bounding_circles.circles().size() - offset; i++)
		{
			m_circles[i].center = sim.getPedestrians()[i].circle.center;
			m_polygons[i][0] = m_circles[i].center;
			m_polygons[i][1] = sim.getPedestrianNominalPosition(i);
		}
		const Capsule& robot_shape(sim.getRobot().rds_configuration.robot_shape);
		Vec2 v_result;
		sim.getRobot().transformVectorLocalToGlobal(robot_shape.center_a(), &v_result);
		Vec2 center_a = v_result + sim.getRobot().position;
		sim.getRobot().transformVectorLocalToGlobal(robot_shape.center_b(), &v_result);
		Vec2 center_b = v_result + sim.getRobot().position;
		m_capsules[0] = Capsule(m_capsules[0].radius(), center_a, center_b);

		sim.getRobot().transformVectorLocalToGlobal(sim.getRobot().rds_configuration.p_ref, &v_result);
		Vec2 p_ref_global = v_result + sim.getRobot().position;
		m_polygons.back()[0] = p_ref_global;
		sim.m_crowd_trajectory.getPedestrianPositionAtTime(sim.m_robot_leader_index, sim.getTime(), &v_result);
		m_polygons.back()[1] = v_result;
		Vec2 nominal_position = v_result;

		update_mean((nominal_position - p_ref_global).norm());

		for (std::vector<Circle>::size_type i = m_circles.size() - m_bounding_circles.circles().size() - offset; i < m_circles.size() - offset; i++)
		{
			unsigned int bc_index = i - (m_circles.size() - m_bounding_circles.circles().size() - offset);
			sim.getRobot().transformVectorLocalToGlobal(m_bounding_circles.circles()[bc_index].center, &v_result);
			m_circles[i] = Circle(v_result + sim.getRobot().position, m_bounding_circles.circles()[bc_index].radius);
		}
		if (sim.m_orca_orca)
			m_circles.back() = sim.getOrcaOrcaCircle();

		if ((m_track_from_time >= 0.f) && sim.getTime() > m_track_from_time)
		{
			GuiColor red, white;
			red.g = red.b = 0;
			m_points.push_back(p_ref_global);
			m_gui.points_colors.push_back(white);
			m_points.push_back(nominal_position);
			m_gui.points_colors.push_back(red);
		}
		return (m_gui.update() == 0);
	}

	GUI m_gui;
	std::vector<Vec2> m_points;
	float m_track_from_time;
	std::vector<Circle> m_circles;
	std::vector<Capsule> m_capsules;
	const BoundingCircles& m_bounding_circles;
	std::vector<Polygon> m_polygons;
};

int pseudo_main(int argc, char** argv)
{
	char file_name[] = "./data_university_students/students003_no_obstacles.vsp";
	float frame_rate = 25.333;
	float scaling = 0.025;//0.027;
	CrowdTrajectory crowd_trajectory(file_name, frame_rate, scaling);

	unsigned int robot_leader_index = 23;
	if ((argc > 1) && (std::stoi(argv[1]) >= 0))
		robot_leader_index = std::stoi(argv[1]);
	bool orca_orca_version = false;
	if (argc > 2)
		orca_orca_version = (std::stoi(argv[2]) > 0);
	float track_from_time = -1.f;
	if (argc > 3)
		track_from_time = std::stod(argv[3]);
	float termination_time = 100000.f;
	if (argc > 4)
		termination_time = std::stod(argv[4]);
	bool auto_termination_time = false;
	if (argc > 5)
		auto_termination_time = true;

	crowd_trajectory.m_time_shift = crowd_trajectory.getSplinesData()[robot_leader_index][0].t;

	if (auto_termination_time)
	{
		termination_time = crowd_trajectory.getSplinesData()[robot_leader_index].back().t;
		termination_time += 2.f;
		termination_time -= crowd_trajectory.m_time_shift;
	}

	float y_ref = 0.2f;
	float y_front_circle = 0.1f;
	RDSCapsuleConfiguration config(1.f, 0.05f, 1.7f,
		Capsule(0.4f, Vec2(0.f, y_front_circle), Vec2(0.f, -0.3f)), Vec2(0.f, y_ref));
	CrowdRdsOrcaSimulator sim(config, crowd_trajectory, robot_leader_index, orca_orca_version);

	for (unsigned int i = 0; i < crowd_trajectory.getNumSplines(); i++)
	{
		if (i != robot_leader_index)
			sim.addPedestrian(i);
	}

	sim.m_robot_avoids = true;

	GuiWrap gui_wrap(sim, track_from_time);

	std::chrono::milliseconds gui_cycle_time(int(dt*1000.f));
	std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
	do
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
		sim.step(dt);
		std::cout << sim.getTime() << "\t\r" << std::flush;
	}
	while (gui_wrap.update(sim) && (sim.getTime() < termination_time));

	std::cout << "Robot mean distance to target = ";
	std::cout << robot_mean_distance_to_target << std::endl;

	return 0;
}

int main(int argc, char** argv)
{
	if (argc > 1)
		return pseudo_main(argc, argv);

	double average_distance_to_target_rds_orca = 0.0;
	double average_distance_to_target_orca_orca = 0.0;
	char pseudo_argv_0[] = "dummy";
	char pseudo_argv_1[5];
	char pseudo_argv_2_a[] = {"0"};
	char pseudo_argv_2_b[] = {"1"};
	char pseudo_argv_3[] = {"0"};
	char pseudo_argv_4[] = {"20"};
	char* pseudo_argv_a[] = {pseudo_argv_0, pseudo_argv_1, pseudo_argv_2_a, pseudo_argv_3, pseudo_argv_4, pseudo_argv_0};
	char* pseudo_argv_b[] = {pseudo_argv_0, pseudo_argv_1, pseudo_argv_2_b, pseudo_argv_3, pseudo_argv_4, pseudo_argv_0};

	for (int i = 0; i < 100; i++)
	{
		std::sprintf(pseudo_argv_1, "%d", i);

		robot_mean_distance_to_target = 0.0;
		time_counter = 0.0;
		pseudo_main(6, pseudo_argv_a);
		average_distance_to_target_rds_orca = average_distance_to_target_rds_orca*i/double(i + 1) + robot_mean_distance_to_target/(i + 1);

		robot_mean_distance_to_target = 0.0;
		time_counter = 0.0;
		pseudo_main(6, pseudo_argv_b);
		average_distance_to_target_orca_orca = average_distance_to_target_orca_orca*i/double(i + 1) + robot_mean_distance_to_target/(i + 1);
	}
	std::cout << "average_distance_to_target_rds_orca = ";
	std::cout << average_distance_to_target_rds_orca << std::endl;
	std::cout << "average_distance_to_target_orca_orca = ";
	std::cout << average_distance_to_target_orca_orca << std::endl;
}

/*
For this code I got (after 100 simulation cases):
average_distance_to_target_rds_orca = 0.96995
average_distance_to_target_orca_orca = 1.44813
*/