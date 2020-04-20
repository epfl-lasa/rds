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

const float dt = 0.005f;

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
			m_gui.circles_colors.push_back(GuiColor());
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
			if (sim.getRobotCollisions()[i])
				m_gui.circles_colors[i].g = m_gui.circles_colors[i].b = 0;
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

int main(int argc, char** argv)
{
	CrowdTrajectory crowd_trajectory;
	{
		std::vector<CrowdTrajectory::Knot> spline_data(3);
		Vec2 move(5.15f, -2.15f);
		std::vector<Vec2> static_pedestrian_positions = {move + Vec2(-0.3f,-0.3f),
			move + Vec2(-0.3f, 0.3f), move + Vec2(0.3f,-0.3f), move + Vec2(0.3f, 0.3f)};
		Vec2 shift_per_second(-1.3f, 0.f);
		shift_per_second = Vec2();
		for (auto& ped_pos : static_pedestrian_positions)
		{
			spline_data[0] = CrowdTrajectory::Knot(ped_pos, 0.f);
			spline_data[1] = CrowdTrajectory::Knot(ped_pos + shift_per_second, 1.f);
			spline_data[2] = CrowdTrajectory::Knot(ped_pos + 2.f*shift_per_second, 2.f);
			crowd_trajectory.addPedestrianTrajectory(spline_data);
		}
	}

	unsigned int robot_leader_index = crowd_trajectory.getSplinesData().size();
	std::vector<CrowdTrajectory::Knot> spline_data(3);
	spline_data[0] = CrowdTrajectory::Knot(Vec2(-5.f, -2.f), 0.f);
	spline_data[1] = CrowdTrajectory::Knot(Vec2(-3.7f, -2.f), 1.f);
	spline_data[2] = CrowdTrajectory::Knot(Vec2(-2.4f, -2.f), 2.f);
	crowd_trajectory.addPedestrianTrajectory(spline_data);

	float y_ref = 0.2f;
	float y_front_circle = 0.1f;
	RDSCapsuleConfiguration config(0.5f, 0.05f, 2.0f,
		Capsule(0.4f, Vec2(0.f, y_front_circle), Vec2(0.f, -0.3f)), Vec2(0.f, y_ref));
	CrowdRdsOrcaSimulator sim(config, crowd_trajectory, robot_leader_index, false);

	for (unsigned int i = 0; i < crowd_trajectory.getNumSplines(); i++)
	{
		if (i != robot_leader_index)
			sim.addPedestrian(i);
	}

	sim.m_robot_avoids = false;

	GuiWrap gui_wrap(sim, 0.f);

	float termination_time = 20.f;

	std::chrono::milliseconds gui_cycle_time(int(dt*1000.f));
	std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
	do
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
		sim.step(dt);
		sim.checkRobotCollisions();
		unsigned int collision_count = 0;
		for (const auto& collision : sim.getRobotCollisions())
			collision_count += int(collision);
		char time_str[20];
		std::sprintf(time_str, "%6.2f", sim.getTime());
		std::cout << "Time=" << time_str << "; Collisions=" << collision_count << "\t\r" << std::flush;
	}
	while (gui_wrap.update(sim) && (sim.getTime() < termination_time));

	std::cout << "Robot mean distance to target = ";
	std::cout << robot_mean_distance_to_target << std::endl;

	return 0;
}