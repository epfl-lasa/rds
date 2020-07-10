#include "config_rds_5.hpp"

#include "gui.hpp"
#include "rds_5_orca_simulator.hpp"
#include "rds_5_agent.hpp"
#include "vw_limits.hpp"
#include "geometry.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdio>
#include <fstream>
#include <vector>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;
using Geometry2D::BoundingCircles;
using AdditionalPrimitives2D::Polygon;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;

const bool with_gui = true;
const bool save_result = true;

const float dt = 0.05f;

const RDS5CapsuleConfiguration rds_5_config = ConfigRDS5::ConfigWrap(dt).rds_5_config;

const float robot_closeness_threshold = 3.f;
const float goal_reaching_threshold = 0.5f;

struct AgentLog
{
	AgentLog() : v_mean(0), time_when_finishing(-1), distance_to_target_mean(0),
		time_insde_arena(0), time_close_to_robot(0), duration_distance_to_target_mean(0) { }
	double v_mean, time_when_finishing, distance_to_target_mean, time_insde_arena, time_close_to_robot;
	double duration_distance_to_target_mean;
};

struct Arena
{
	float x_min, x_max, y_min, y_max;
	bool contains(const Vec2& p) const
	{
		return (p.x > x_min) && (p.x < x_max) && (p.y > y_min) && (p.y < y_max);
	}
};

void define_arena_for_evaluation(const CrowdTrajectory& c, Arena& a)
{
	a.x_min = a.y_min = 100000.0f;
	a.x_max = a.y_max = -100000.0f;
	for (const auto& s : c.getSplinesData())
	{
		for (const auto& k : s)
		{
			if (k.p.x < a.x_min)
				a.x_min = k.p.x;
			if (k.p.x > a.x_max)
				a.x_max = k.p.x;
			if (k.p.y < a.y_min)
				a.y_min = k.p.y;
			if (k.p.y > a.y_max)
				a.y_max = k.p.y;
		}
	}
}

void update_mean(double *mean, double value, double time)
{
	double w_old = time/(time + dt);
	double w_new = dt/(time + dt);
	*mean = w_old*(*mean) + w_new*value;
}

void update_mean_seasonally(double *mean, double value, double* time_accumulation)
{
	double time = *time_accumulation;
	double w_old = time/(time + dt);
	double w_new = dt/(time + dt);
	*mean = w_old*(*mean) + w_new*value;
	*time_accumulation = *time_accumulation + dt;
}

GuiColor blue_green_color(float weight_blue)
{
	GuiColor this_color;
	this_color.r = 0;
	weight_blue = std::min(0.999999f, std::max(0.000001f, weight_blue));
	this_color.b = 255*weight_blue;
	this_color.g = 255*(1.f - weight_blue);
	return this_color;
}

struct GuiWrap
{
	GuiWrap(const CrowdRdsOrcaSimulator& sim, float track_from_time)
	: m_constraints_gui("Constraints of RDS-ORCA Simulator",
		sim.getRobot().rds_configuration.vw_diamond_limits.v_max*2.f,
		1200, !(with_gui))
	, m_gui("RDS-ORCA Simulator", 20.f, 1200, !(with_gui))
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
		if (sim.getRobot().ORCA_implementation)
		{
			Circle tight_bounding_circle;
			sim.getRobot().computeTightBoundingCircle(&tight_bounding_circle);
			m_circles.push_back(tight_bounding_circle);
		}

		m_constraints_gui.halfplanes = &m_constraints;
		m_constraints_gui.arrows = &m_arrows;
	}

	bool update_gui_and_log(const CrowdRdsOrcaSimulator& sim,
		std::vector<AgentLog>& crowd_log, AgentLog& robot_log, const Arena& arena, float t_final)
	{
		int offset = 0;
		if (sim.m_orca_orca)
			offset += 1;
		if (sim.getRobot().ORCA_implementation)
			offset += 1;
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

		for (std::vector<Circle>::size_type i = m_circles.size() - m_bounding_circles.circles().size() - offset; i < m_circles.size() - offset; i++)
		{
			unsigned int bc_index = i - (m_circles.size() - m_bounding_circles.circles().size() - offset);
			sim.getRobot().transformVectorLocalToGlobal(m_bounding_circles.circles()[bc_index].center, &v_result);
			m_circles[i] = Circle(v_result + sim.getRobot().position, m_bounding_circles.circles()[bc_index].radius);
		}
		if (sim.m_orca_orca && !sim.getRobot().ORCA_implementation)
			m_circles.back() = sim.getOrcaOrcaCircle();
		else if (sim.getRobot().ORCA_implementation && !sim.m_orca_orca)
			sim.getRobot().computeTightBoundingCircle(&m_circles.back());
		else if (sim.m_orca_orca && sim.getRobot().ORCA_implementation)
		{
			m_circles[m_circles.size() - 2] = sim.getOrcaOrcaCircle();
			sim.getRobot().computeTightBoundingCircle(&m_circles.back());
		}

		if ((m_track_from_time >= 0.f) && sim.getTime() > m_track_from_time)
		{
			GuiColor red, reddish_white;
			red.g = red.b = 0;
			reddish_white.g = reddish_white.b = 190;
			m_points.push_back(p_ref_global);
			m_gui.points_colors.push_back(red);
			m_points.push_back(nominal_position);
			m_gui.points_colors.push_back(red);
		}

		m_constraints = sim.getRobot().constraints;
		m_arrows.resize(0);
		m_arrows.push_back(Arrow(sim.getRobot().last_step_p_ref_velocity_local, Vec2(0.f, 0.f)));
		m_arrows.push_back(Arrow(sim.getRobot().last_step_nominal_p_ref_velocity_local, Vec2(0.f, 0.f)));

		// update logs
		double t = sim.getTime() - dt;
		for (unsigned int i = 0; i != sim.getPedestrianIndices().size(); ++i)
		{
			unsigned int pedestrian_index = sim.getPedestrianIndices()[i];
			AgentLog& p_log = crowd_log[pedestrian_index];
			
			Vec2 target;
			sim.m_crowd_trajectory.getPedestrianPositionAtTime(pedestrian_index, t, &target);
			const Vec2& position = sim.getPedestrians()[i].circle.center;
			double distance_to_target = (target - position).norm();
			//update_mean(&(p_log.distance_to_target_mean), distance_to_target, t);
			if (arena.contains(target))
			{
				update_mean_seasonally(&(p_log.distance_to_target_mean), distance_to_target, &(p_log.duration_distance_to_target_mean));
				if ((m_track_from_time >= 0.f) && sim.getTime() > m_track_from_time)
				{
					GuiColor this_color = blue_green_color(float(i)/sim.getPedestrianIndices().size());
					//float w_g = (255 - this_color.g)/255.0;
					//float w_b = (255 - this_color.b)/255.0;
					//float w_r = 1.0;
					//float w_sum = w_r + w_g + w_b;
					//GuiColor this_whitish_color = this_color;
					//this_whitish_color.r = 190;
					m_points.push_back(position);
					m_gui.points_colors.push_back(this_color);
					m_points.push_back(target);
					m_gui.points_colors.push_back(this_color);
				}
			}

			if (p_log.time_when_finishing < 0.0)
				update_mean(&(p_log.v_mean), sim.getPedestrians()[i].velocity.norm(), t);

			if (arena.contains(position))
				p_log.time_insde_arena += dt;

			if ((p_log.time_when_finishing < 0.0) && (t > t_final) && (distance_to_target < goal_reaching_threshold))
				p_log.time_when_finishing = t;

			float distance_to_robot = (sim.m_previous_robot_position - position).norm();
			if (distance_to_robot < robot_closeness_threshold)
				p_log.time_close_to_robot += dt;
		}
		if (robot_log.time_when_finishing < 0.0)
			update_mean(&(robot_log.v_mean), sim.getRobot().last_step_p_ref_velocity.norm(), t);
		float distance_to_target = (nominal_position - p_ref_global).norm();
		update_mean(&(robot_log.distance_to_target_mean), distance_to_target, t);
		if ((robot_log.time_when_finishing < 0.0) && (t > t_final) && (distance_to_target < goal_reaching_threshold))
			robot_log.time_when_finishing = t;
		if (arena.contains(p_ref_global))
			robot_log.time_insde_arena += dt;

		if (with_gui)
			return (m_gui.update() == 0) | (m_constraints_gui.update() == 0);
		else
			return true;
	}

	GUI m_constraints_gui, m_gui;
	std::vector<Vec2> m_points;
	float m_track_from_time;
	std::vector<Circle> m_circles;
	std::vector<Capsule> m_capsules;
	const BoundingCircles& m_bounding_circles;
	std::vector<Polygon> m_polygons;
	std::vector<HalfPlane2> m_constraints;
	std::vector<Arrow> m_arrows;
};

CrowdRdsOrcaSimulator* setup_simulation(CrowdTrajectory* crowd_motion,
	int robot_index, int mode)
{
	float robot_t_data_start = crowd_motion->getSplinesData()[robot_index][0].t;
	float robot_t_data_end = crowd_motion->getSplinesData()[robot_index].back().t;
	crowd_motion->m_time_shift = robot_t_data_start;
	crowd_motion->m_duration = robot_t_data_end - robot_t_data_start;

	int handy_robot_index = (mode == 0) ? 666 : robot_index;
	bool orca_orca = false;//(mode == 1);
	CrowdRdsOrcaSimulator* simulation = new CrowdRdsOrcaSimulator(rds_5_config,
		*crowd_motion, handy_robot_index, orca_orca);
	if (mode == 1)
	{
		bool using_p_ref_as_control_point = true; // = using large circle
		simulation->implementORCA(using_p_ref_as_control_point);
		simulation->useORCASolver();
	}
	//simulation->useDefaultNominalCommand(Vec2(0.f, 1.3f));

	simulation->m_ignore_orca_circle = false;

	if (mode == 0)
		simulation->addPedestrian(robot_index);
	for (unsigned int i = 0; i != crowd_motion->getNumSplines(); i++)
	{
		if (i != robot_index)
			simulation->addPedestrian(i);
	}
	return simulation;
}

void crowd_sample(unsigned int sample_index, int* robot_index, std::vector<unsigned int>* pedestrian_indices)
{
	*robot_index = sample_index*3;
	pedestrian_indices->resize(32);
	for (unsigned int i = 0; i != pedestrian_indices->size(); i++)
		(*pedestrian_indices)[i] = *robot_index + 1 + i;
}

double compute_crowd_tracking_error(const std::vector<AgentLog>& crowd_log)
{
	double weight_sum = 0.0;
	double weighted_mean = 0.0;
	for (const auto& p_log : crowd_log)
	{
		double weight = p_log.duration_distance_to_target_mean;
		weighted_mean += weight*p_log.distance_to_target_mean;
		weight_sum += weight;
	}
	return weighted_mean/weight_sum;
}

int main()
{
	CrowdTrajectory crowd_trajectory;
	{
		std::vector<CrowdTrajectory::Knot> spline_data(3);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-0.15f + 0.5f*k*7, -1.f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-0.15f + 0.5f*k*7, -2.f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-0.15f + 0.5f*k*7, -3.f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-1.5f + 0.5f*k*7, -4.f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-0.5f + 0.5f*k*7, 0.2f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-0.5f + 0.5f*k*7, 1.f), float(k*7));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
				spline_data[k] = CrowdTrajectory::Knot(Vec2(-5.6f + j, 4.5f + j*1.2f)+k*7*Vec2(0.7f, -0.7f), float(k*7));
			crowd_trajectory.addPedestrianTrajectory(spline_data);
			for (int k = 0; k < 3; k++)
				spline_data[k] = CrowdTrajectory::Knot(Vec2(-5.2f + j, -4.5f - j*1.2f)+k*7*Vec2(0.7f, 0.7f), float(k*7));
			crowd_trajectory.addPedestrianTrajectory(spline_data);
		}
	}
	unsigned int robot_leader_index = crowd_trajectory.getSplinesData().size();
	std::vector<CrowdTrajectory::Knot> spline_data(3);
	for (int k = 0; k < 3; k++)
		spline_data[k] = CrowdTrajectory::Knot(Vec2(-8.f + 1.3f*k*9, 0.f), float(k*9));
	crowd_trajectory.addPedestrianTrajectory(spline_data);

	Arena arena;
	arena.x_min = -20.f;
	arena.x_max = 20.f;
	arena.y_min = -20.f;
	arena.y_max = 20.f;

	std::vector<double> RDS_E_t, RDS_E_v, RDS_N_ttg, RDS_N_v,
		ORCA_E_t, ORCA_E_v, ORCA_N_ttg, ORCA_N_v,
		RDS_robot_mean_distance_to_target, ORCA_robot_mean_distance_to_target,
		RDS_ped_mean_distance_to_target, ORCA_ped_mean_distance_to_target;

	std::vector<unsigned int> RDS_collision_count, ORCA_collision_count;

	int robot_index = robot_leader_index;
	CrowdRdsOrcaSimulator* sim;
	{
		double reaching_time_crowd[3], velocity_crowd[3];
		for (int mode = 1; mode != 3; ++mode)
		{
			sim = setup_simulation(&crowd_trajectory, robot_index, mode);

			std::vector<AgentLog> crowd_log(crowd_trajectory.getNumSplines());
			AgentLog robot_log;

			float t_final = crowd_trajectory.m_duration;
			float t_termination = 1.0f*t_final;

			GuiWrap gui_wrap(*sim, 0.f); // draw traces from t=0 on
			std::chrono::milliseconds gui_cycle_time(int(dt*1000.f));
			std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
			
			unsigned int collision_count;
			unsigned int beginner_collision_count = 0;
			do
			{
				if (with_gui)
				{
					std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::high_resolution_clock::now() - t_gui_update));
				}
				t_gui_update = std::chrono::high_resolution_clock::now();
				sim->step(dt);
				sim->checkRobotCollisions();
				//unsigned int 
				collision_count = 0;
				for (const auto& collision : sim->getRobotCollisions())
					collision_count += int(collision);

				if (sim->getTime() - dt < 1.f)
				{
					beginner_collision_count = 0;
					for (const auto& collision : sim->getRobotCollisions())
						beginner_collision_count += int(collision);
				}

				char time_str[20];
				std::sprintf(time_str, "%6.2f", sim->getTime());
				std::cout << "Time=" << time_str << "; Collisions=" << collision_count << "\t\r" << std::flush;
			}
			while (gui_wrap.update_gui_and_log(*sim, crowd_log, robot_log, arena, t_final) &&
				((sim->getTime() - dt) < t_termination));

			// evaluate metrics
			double arena_time_sum = 0.0;
			double close_to_robot_time_sum = 0.0;
			for (const auto& p_log : crowd_log)
			{
				arena_time_sum += p_log.time_insde_arena;
				close_to_robot_time_sum += p_log.time_close_to_robot;
			}
			reaching_time_crowd[mode] = 0.0;
			velocity_crowd[mode] = 0.0;
			for (const auto& p_log : crowd_log)
			{
				double weight = p_log.time_insde_arena/arena_time_sum;
				if (p_log.time_when_finishing > 0.0)
					reaching_time_crowd[mode] += weight*p_log.time_when_finishing;
				else
					reaching_time_crowd[mode] += weight*t_termination;
				velocity_crowd[mode] += weight*p_log.v_mean;
			}
			double N_ttg = 0.0;
			double N_v = 0.0;
			for (auto& p_log : crowd_log)
			{
				double weight = p_log.time_close_to_robot/close_to_robot_time_sum;
				if (p_log.time_when_finishing > 0.0)
					N_ttg += weight*p_log.time_when_finishing;
				else
					N_ttg += weight*t_termination;
				N_v += weight*p_log.v_mean;
			}
			N_ttg /= reaching_time_crowd[mode];
			N_v /= velocity_crowd[mode];

			if (mode == 1)
			{
				ORCA_N_ttg.push_back(N_ttg);
				ORCA_N_v.push_back(N_v);
				ORCA_robot_mean_distance_to_target.push_back(robot_log.distance_to_target_mean);
				ORCA_collision_count.push_back(collision_count - beginner_collision_count);
				ORCA_ped_mean_distance_to_target.push_back(compute_crowd_tracking_error(crowd_log));
			}
			else if (mode == 2)
			{
				RDS_N_ttg.push_back(N_ttg);
				RDS_N_v.push_back(N_v);
				RDS_robot_mean_distance_to_target.push_back(robot_log.distance_to_target_mean);
				RDS_collision_count.push_back(collision_count - beginner_collision_count);
				RDS_ped_mean_distance_to_target.push_back(compute_crowd_tracking_error(crowd_log));
			}
			delete sim;
		}
		RDS_E_t.push_back(reaching_time_crowd[0]/reaching_time_crowd[2]);
		ORCA_E_t.push_back(reaching_time_crowd[0]/reaching_time_crowd[1]);
		RDS_E_v.push_back(velocity_crowd[0]/velocity_crowd[2]);
		ORCA_E_v.push_back(velocity_crowd[0]/velocity_crowd[1]);
	}

	std::vector<std::string> metric_names = {
		"RDS_E_t           ",
		"ORCA_E_t          ",
		"RDS_E_v           ",
		"ORCA_E_v          ",
		"RDS_N_ttg         ",
		"ORCA_N_ttg        ",
		"RDS_N_v           ",
		"ORCA_N_v          ",
		"RDS_rob_track_err ",
		"ORCA_rob_track_err",
		"RDS_ped_track_err ",
		"ORCA_ped_track_err"
	};
	std::vector<std::string> integer_metric_names = {
		"RDS_collisions    ",
		"ORCA_collisions   "};
	std::vector<std::vector<double> > metric_values = {RDS_E_t, ORCA_E_t, RDS_E_v, ORCA_E_v,
		RDS_N_ttg, ORCA_N_ttg, RDS_N_v, ORCA_N_v,
		RDS_robot_mean_distance_to_target, ORCA_robot_mean_distance_to_target,
		RDS_ped_mean_distance_to_target, ORCA_ped_mean_distance_to_target
	};
	std::vector<std::vector<unsigned int> > integer_metric_values = {RDS_collision_count, ORCA_collision_count};
	
	std::vector<double> metric_mean_values(metric_values.size(), 0.0);
	for (unsigned int i = 0; i < metric_values.size(); i++)
	{
		for (unsigned int j = 0; j < metric_values[i].size(); j++)
			metric_mean_values[i] += metric_values[i][j];
		metric_mean_values[i] /= double(metric_values[i].size());
	}
	std::vector<double> metric_std_values(metric_values.size(), 0.0);
	for (unsigned int i = 0; i < metric_values.size(); i++)
	{
		for (unsigned int j = 0; j < metric_values[i].size(); j++)
		{
			double diff = metric_values[i][j] - metric_mean_values[i];
			metric_std_values[i] += diff*diff;
		}
		metric_std_values[i] = std::sqrt(metric_std_values[i]/(metric_values[i].size() - 1));
	}

	std::vector<double> integer_metric_mean_values(integer_metric_values.size(), 0.0);
	for (unsigned int i = 0; i < integer_metric_values.size(); i++)
	{
		for (unsigned int j = 0; j < integer_metric_values[i].size(); j++)
			integer_metric_mean_values[i] += integer_metric_values[i][j];
		integer_metric_mean_values[i] /= double(integer_metric_values[i].size());
	}
	std::vector<double> integer_metric_std_values(integer_metric_values.size(), 0.0);
	for (unsigned int i = 0; i < integer_metric_values.size(); i++)
	{
		for (unsigned int j = 0; j < integer_metric_values[i].size(); j++)
		{
			double diff = integer_metric_values[i][j] - integer_metric_mean_values[i];
			integer_metric_std_values[i] += diff*diff;
		}
		integer_metric_std_values[i] = std::sqrt(integer_metric_std_values[i]/(integer_metric_values[i].size() - 1));
	}

	for (unsigned int i = 0; i < metric_values.size(); i++)
	{
		char mean_str[20], std_str[20];
		std::sprintf(mean_str, "%10.6f", metric_mean_values[i]);
		std::sprintf(std_str, "%10.6f", metric_std_values[i]);
		std::cout << metric_names[i] << ":   mean =" << mean_str;
		std::cout << "   std =" << std_str << std::endl;
	}
	for (unsigned int i = 0; i < integer_metric_values.size(); i++)
	{
		char mean_str[20], std_str[20];
		std::sprintf(mean_str, "%10.6f", integer_metric_mean_values[i]);
		std::sprintf(std_str, "%10.6f", integer_metric_std_values[i]);
		std::cout << integer_metric_names[i] << ":   mean =" << mean_str;
		std::cout << "   std =" << std_str << std::endl;
	}

	if (save_result)
	{
		std::ofstream csv_file("metrics_evaluation_dynamic_scene_complex.csv", std::ios::trunc);
		csv_file << metric_names[0];
		for (unsigned int j = 1; j != metric_names.size(); j++)
			csv_file << "; " << metric_names[j];
		for (unsigned int j = 0; j != integer_metric_names.size(); j++)
			csv_file << "; " << integer_metric_names[j];
		for (unsigned int i = 0; i != metric_values[0].size(); ++i)
		{
			csv_file << std::endl << metric_values[0][i];
			for (unsigned int j = 1; j != metric_values.size(); j++)
				csv_file << "; " << metric_values[j][i];
			for (unsigned int j = 0; j != integer_metric_values.size(); j++)
				csv_file << "; " << integer_metric_values[j][i];
		}
	}

	return 0;
}