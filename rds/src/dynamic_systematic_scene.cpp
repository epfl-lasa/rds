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
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::BoundingCircles;
using AdditionalPrimitives2D::Polygon;
using AdditionalPrimitives2D::Arrow;

const bool with_gui = true;
const bool save_result = false;
const bool robot_avoids = true;
const bool pedestrian_avoids = false;

const float dt = 0.05f;

RDS5CapsuleConfiguration rds_5_config = ConfigRDS5::ConfigWrap(dt).rds_5_config;

const float robot_closeness_threshold = 3.f;
const float goal_reaching_threshold = 0.5f;

std::vector<Vec2> robot_trace, pedestrian_trace;
std::vector<GuiColor> trace_colors;
unsigned int trace_count = 0;
const unsigned int n_samples = 20;

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

void define_arena_for_evaluation(Arena& a)
{
	a.x_min = a.y_min = -100000.0f;
	a.x_max = a.y_max = 100000.0f;
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
	: m_gui("RDS-ORCA Simulator", 10.f, 1000, !(with_gui))
	, m_constraints_gui("Constraints", sim.getRobot().rds_configuration.vw_diamond_limits.v_max*2.f,
		1200, !(with_gui))
	, m_bounding_circles(sim.getBoundingCirclesRobot())
	, m_track_from_time(track_from_time)
	{
		m_gui.points = &m_points;
		m_gui.circles = &m_circles;
		m_gui.capsules = &m_capsules;
		m_gui.polygons = &m_polygons;
		for (unsigned int i = 0; i < trace_colors.size(); i++) 
		{
			m_points.push_back(robot_trace[i]);
			m_points.push_back(pedestrian_trace[i]);
			m_gui.points_colors.push_back(trace_colors[i]);
			m_gui.points_colors.push_back(trace_colors[i]);
		}
		trace_count++;

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

		GuiColor red, white;
		red.g = red.b = 0;
		if ((m_track_from_time >= 0.f) && sim.getTime() > m_track_from_time)
		{
			m_points.push_back(p_ref_global);
			m_gui.points_colors.push_back(white);
			m_points.push_back(nominal_position);
			m_gui.points_colors.push_back(red);
			robot_trace.push_back(p_ref_global);
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
			if (arena.contains(target) && (t < t_final - 0.75f))
				update_mean_seasonally(&(p_log.distance_to_target_mean), distance_to_target, &(p_log.duration_distance_to_target_mean));

			if ((m_track_from_time >= 0.f) && sim.getTime() > m_track_from_time)
			{
				//float w_g = (255 - this_color.g)/255.0;
				//float w_b = (255 - this_color.b)/255.0;
				//float w_r = 1.0;
				//float w_sum = w_r + w_g + w_b;
				//GuiColor this_whitish_color = this_color;
				//this_whitish_color.r = 190;
				m_points.push_back(position);
				m_gui.points_colors.push_back(white);
				m_points.push_back(target);
				m_gui.points_colors.push_back(red);
				pedestrian_trace.push_back(position);
				trace_colors.push_back(blue_green_color(trace_count/float(n_samples)));
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
		if (t < t_final - 0.75f)
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

	GUI m_gui, m_constraints_gui;
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

	simulation->m_ignore_orca_circle = false;

	if (mode == 0)
		simulation->addPedestrian(robot_index);
	for (unsigned int i = 0; i != crowd_motion->getNumSplines(); i++)
	{
		if (i != robot_index)
		{
			simulation->addPedestrian(i);
			if (!pedestrian_avoids)
				simulation->disableAvoidanceForRecentlyAddedPedestrian();
		}
	}
	simulation->m_robot_avoids = robot_avoids;
	return simulation;
}

void crowd_sample(unsigned int sample_index, int* robot_index, std::vector<unsigned int>* pedestrian_indices)
{
	*robot_index = sample_index*3;
	pedestrian_indices->resize(32);
	for (unsigned int i = 0; i != pedestrian_indices->size(); i++)
		(*pedestrian_indices)[i] = *robot_index + 1 + i;
}

double compute_crowd_tracking_error(const std::vector<AgentLog>& crowd_log,
	const CrowdRdsOrcaSimulator& sim)
{
	double weight_sum = 0.0;
	double weighted_mean = 0.0;
	for (unsigned int i = 0; i != sim.getPedestrianIndices().size(); ++i)
	{
		unsigned int pedestrian_index = sim.getPedestrianIndices()[i];
		const AgentLog& p_log = crowd_log[pedestrian_index];
		double weight = p_log.duration_distance_to_target_mean;
		weighted_mean += weight*p_log.distance_to_target_mean;
		weight_sum += weight;
	}
	if (weight_sum > 0.0)
		return weighted_mean/weight_sum;
	else
		return -1.0;
}

int main()
{

	rds_5_config.linear_acceleration_limit = 1000000.f;
	rds_5_config.angular_acceleration_limit = 1000000.f;
	rds_5_config.vw_diamond_limits = VWDiamond(-10.f, 10.f, 10.f, 0.f);
	Arena arena;
	define_arena_for_evaluation(arena);

	std::vector<double> RDS_E_t, RDS_E_v, RDS_N_ttg, RDS_N_v,
		ORCA_E_t, ORCA_E_v, ORCA_N_ttg, ORCA_N_v,
		RDS_robot_mean_distance_to_target, ORCA_robot_mean_distance_to_target,
		RDS_ped_mean_distance_to_target, ORCA_ped_mean_distance_to_target;

	std::vector<unsigned int> RDS_collision_count, ORCA_collision_count;

	int robot_index;
	CrowdRdsOrcaSimulator* sim;
	//const unsigned int n_samples = 10;
	for (unsigned int sample_index = 0; sample_index != n_samples; ++sample_index)
	{
		robot_index = 0;
		CrowdTrajectory crowd_trajectory;	
		std::vector<CrowdTrajectory::Knot> spline_data(3);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(-5.95f + 1.3f*k*5, 0.f), float(k*5));
		crowd_trajectory.addPedestrianTrajectory(spline_data);
		for (int k = 0; k < 3; k++)
			spline_data[k] = CrowdTrajectory::Knot(Vec2(0.f, -8.f + sample_index/float(n_samples-1)*4.f + 1.3f*k*5), float(k*5));
		crowd_trajectory.addPedestrianTrajectory(spline_data);

		double reaching_time_crowd[3], velocity_crowd[3];
		for (int mode = 2; mode != 3; ++mode)
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

				//char time_str[20];
				//std::sprintf(time_str, "%6.2f", sim->getTime());
				//std::cout << "Time=" << time_str << "; Collisions=" << collision_count << "\t\r" << std::flush;
			}
			while (gui_wrap.update_gui_and_log(*sim, crowd_log, robot_log, arena, t_final) &&
				((sim->getTime() - dt) < t_termination));

			// remove the robot's pedestrian (to evaluate crowd metrics for the same crowd size also in case 0)
			crowd_log.erase(crowd_log.begin() + robot_index);

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
				float crowd_tracking_error = compute_crowd_tracking_error(crowd_log, *sim);
				if (crowd_tracking_error > 0.f)
					ORCA_ped_mean_distance_to_target.push_back(crowd_tracking_error);
				else
				{
					ORCA_ped_mean_distance_to_target.push_back(0.f);
					std::cout << "Could not compute crowd tracking error in: sample " << sample_index << ", ";
					std::cout << "mode " << mode << "." << std::endl;
				}
			}
			else if (mode == 2)
			{
				RDS_N_ttg.push_back(N_ttg);
				RDS_N_v.push_back(N_v);
				RDS_robot_mean_distance_to_target.push_back(robot_log.distance_to_target_mean);
				RDS_collision_count.push_back(collision_count - beginner_collision_count);
				float crowd_tracking_error = compute_crowd_tracking_error(crowd_log, *sim);
				if (crowd_tracking_error > 0.f)
					RDS_ped_mean_distance_to_target.push_back(crowd_tracking_error);
				else
				{
					RDS_ped_mean_distance_to_target.push_back(0.f);
					std::cout << "Could not compute crowd tracking error in: sample " << sample_index << ", ";
					std::cout << "mode " << mode << "." << std::endl;
				}
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
		/*std::string metrics_file_name("metrics_evaluation_systematic.csv");
		if (!robot_avoids)
			metrics_file_name = "metrics_evaluation_systematic_not_avoiding.csv";

		std::ofstream csv_file(metrics_file_name, std::ios::trunc);
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
		}*/

		std::string traces_file_name("traces_dynamic_systematic.csv");
		std::ofstream traces_csv_file(traces_file_name, std::ios::trunc);
		GuiColor previous_color(trace_colors[0]);
		unsigned int trace_index = 0;
		for (unsigned int i = 0; i != robot_trace.size(); i++)
		{
			if (!((previous_color.r == trace_colors[i].r) &&
				(previous_color.g == trace_colors[i].g) &&
				(previous_color.b == trace_colors[i].b)))
			{
				previous_color.r = trace_colors[i].r;
				previous_color.g = trace_colors[i].g;
				previous_color.b = trace_colors[i].b;
				trace_index++;
			}
			traces_csv_file << robot_trace[i].x << "; ";
			traces_csv_file << robot_trace[i].y << "; ";
			traces_csv_file << pedestrian_trace[i].x << "; ";
			traces_csv_file << pedestrian_trace[i].y << "; ";
			traces_csv_file << trace_index << std::endl;
		}
	}

	return 0;
}