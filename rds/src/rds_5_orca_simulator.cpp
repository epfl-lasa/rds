#include "rds_5_orca_simulator.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using AdditionalPrimitives2D::Polygon;

#include <iostream>

static RVO::Vector2 toRVO(const Vec2& v)
{
	return RVO::Vector2(v.x, v.y);
}

static Vec2 toRDS(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

static Polygon init_polygon(const Vec2& position, float radius, float max_error)
{
	if (max_error < 0.01f)
		max_error = 0.01f;
	unsigned int n_points = 3;
	while (radius*(1.f - std::cos(M_PI/n_points)) > max_error)
		n_points++;
	Polygon polygon(n_points);
	float center_angles = 2.f*M_PI/n_points;
	for (unsigned int i = 0; i < n_points; i++)
		polygon[i] = position + radius*Vec2(std::cos(i*center_angles), std::sin(i*center_angles));
	return polygon;
}

static std::vector<RVO::Vector2> init_rvo_polygon(const Polygon& polygon)
{
	std::vector<RVO::Vector2> rvo_polygon;
	for (const auto& v : polygon)
		rvo_polygon.push_back(toRVO(v));
	return rvo_polygon;
}

StaticCircle::StaticCircle(const Vec2& position, float radius, float max_error)
	: circle(position, radius)
	, polygon(init_polygon(position, radius, max_error))
	, rvo_polygon(init_rvo_polygon(polygon))
	, robot_collision(false)
{}

RdsOrcaSimulator::RdsOrcaSimulator(const Vec2& position, float orientation,
	const RDS5CapsuleConfiguration& config, const Vec2& reference_point_velocity,
	bool orca_orca)
	: m_bounding_circles_robot(2)
	, m_time(0.f)
	, m_orca_time_horizon(config.tau)
	, m_orca_distance_margin(config.delta)
	, m_pedestrian_radius(0.3f)
	, m_pedestrian_v_max(config.vw_diamond_limits.v_max*1.3)
	, m_orca_orca(orca_orca)
	, m_robot_avoids(true)
	, m_ignore_orca_circle(false)
{
	for (auto& c : m_bounding_circles_robot.circles())
		m_rvo_simulator.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon, 1.f, 1.f);
	if (m_orca_orca)
	{
		m_rvo_simulator.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon, 1.f, 1.f);
		std::vector<size_t> ignore_ids;
		for (unsigned int i = 0; i < m_bounding_circles_robot.circles().size(); i++)
			ignore_ids.push_back(m_rvo_simulator.getAgentID(i));
		m_rvo_simulator.setAgentIgnoreIDs(m_bounding_circles_robot.circles().size(), ignore_ids);
	}
	setRobotProperties(position, orientation, config, reference_point_velocity);
}

unsigned int RdsOrcaSimulator::addPedestrian(const Vec2& position, const Vec2& velocity)
{
	unsigned int agent_no = m_rvo_simulator.addAgent(toRVO(position), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon,
		m_pedestrian_radius + m_orca_distance_margin/2.f, m_pedestrian_v_max, toRVO(velocity));
	if (m_orca_orca && m_ignore_orca_circle)
	{
		std::vector<size_t> ignore_ids = {
			m_rvo_simulator.getAgentID(m_bounding_circles_robot.circles().size()) };
		m_rvo_simulator.setAgentIgnoreIDs(m_rvo_simulator.getNumberOfAgents() - 1, ignore_ids);
	}
	m_pedestrians.push_back(MovingCircle(Circle(position, m_pedestrian_radius), Vec2()));
	m_robot_collisions.push_back(false);
	return agent_no;
}

void RdsOrcaSimulator::setRobotProperties(const Vec2& position, float orientation,
	const RDS5CapsuleConfiguration& config, const Vec2& reference_point_velocity)
{
	m_robot = RDS5CapsuleAgent(position, orientation, config);

	m_bounding_circles_robot.fit(config.robot_shape, m_pedestrian_radius);

	for (std::vector<Circle>::size_type i = 0; i < m_bounding_circles_robot.circles().size(); i++)
	{
		const Vec2& p_local(m_bounding_circles_robot.circles()[i].center);
		Vec2 v_global;
		// set RVO agent's position
		m_robot.transformVectorLocalToGlobal(p_local, &v_global);
		Vec2 p_global(v_global + m_robot.position);
		m_rvo_simulator.setAgentPosition(i, toRVO(p_global));
		// set RVO agent's velocity
		m_robot.transformReferencePointVelocityToPointVelocity(p_local, reference_point_velocity, &v_global);
		m_rvo_simulator.setAgentVelocity(i, toRVO(v_global));
		// set RVO agent's radius
		float radius = m_bounding_circles_robot.circles()[i].radius;
		m_rvo_simulator.setAgentRadius(i, radius + m_orca_distance_margin/2.f);
		// set RVO agent's max speed
		m_rvo_simulator.setAgentMaxSpeed(i, std::max(config.vw_diamond_limits.v_max,
			std::sqrt(config.vw_diamond_limits.w_abs_max*p_local.y*config.vw_diamond_limits.w_abs_max*p_local.y +
				config.vw_diamond_limits.v_at_w_abs_max*config.vw_diamond_limits.v_at_w_abs_max)));
	}
	if (m_orca_orca)
	{
		unsigned int i = m_bounding_circles_robot.circles().size();
		Vec2 v_global;
		m_robot.transformVectorLocalToGlobal(config.p_ref, &v_global);
		m_rvo_simulator.setAgentPosition(i, toRVO(m_robot.position + v_global));
		float radius = config.robot_shape.radius() + std::max(
			(config.robot_shape.center_a() - config.p_ref).norm(),
			(config.robot_shape.center_b() - config.p_ref).norm());
		m_rvo_simulator.setAgentRadius(i, radius + m_orca_distance_margin/2.f);
		
		float diamond_area = 2.f*config.p_ref.y*config.vw_diamond_limits.w_abs_max*(
			config.vw_diamond_limits.v_max - config.vw_diamond_limits.v_at_w_abs_max);
		float v_max = std::sqrt(diamond_area/M_PI);
		v_max = v_max*0.5 + 0.5*std::max(config.vw_diamond_limits.v_max,
			std::sqrt(config.vw_diamond_limits.w_abs_max*config.p_ref.y*config.vw_diamond_limits.w_abs_max*config.p_ref.y +
				config.vw_diamond_limits.v_at_w_abs_max*config.vw_diamond_limits.v_at_w_abs_max));
		m_rvo_simulator.setAgentMaxSpeed(i, v_max);
	}
	/*Vec2 v_pos_to_p_ref_global;
	m_robot.transformVectorLocalToGlobal(config.p_ref, &v_pos_to_p_ref_global);
	m_rvo_simulator.setAgentPosition(0, toRVO(position + v_pos_to_p_ref_global));
	float radius = config.robot_shape.radius() + std::max(
		(config.robot_shape.center_a() - config.p_ref).norm(),
		(config.robot_shape.center_b() - config.p_ref).norm());
	m_rvo_simulator.setAgentRadius(0, radius + m_orca_distance_margin/2.f);
	m_rvo_simulator.setAgentVelocity(0, toRVO(reference_point_velocity));
	m_rvo_simulator.setAgentMaxSpeed(0, config.v_max);*/
}

void RdsOrcaSimulator::step(float dt)
{	
	m_previous_robot_position = m_robot.position;
	int offset = 0;
	if (m_orca_orca)
		offset = 1;
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_pedestrians[i].circle.center = toRDS(m_rvo_simulator.getAgentPosition(i + offset +
			m_bounding_circles_robot.circles().size()));

	//m_rvo_simulator.setAgentPrefVelocity(0, toRVO(getRobotNominalVelocity()));
	for (std::vector<Circle>::size_type i = 0; i < m_bounding_circles_robot.circles().size(); i++)
	{
		const Vec2& p_local(m_bounding_circles_robot.circles()[i].center);
		Vec2 v_global;
		// set RVO agent's preferred velocity
		m_robot.transformReferencePointVelocityToPointVelocity(p_local, getRobotNominalVelocity(), &v_global);
		m_rvo_simulator.setAgentPrefVelocity(i, toRVO(v_global));
	}
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_rvo_simulator.setAgentPrefVelocity(i + offset + m_bounding_circles_robot.circles().size(), getPedestrianNominalVelocity(i));

	if (m_orca_orca)
	{
		unsigned int i = m_bounding_circles_robot.circles().size();
		m_rvo_simulator.setAgentPrefVelocity(i, toRVO(getRobotNominalVelocity()));
	}

	if (!m_ignore_orca_circle && !m_orca_orca && m_robot.ORCA_implementation && m_robot.ORCA_use_p_ref)
	{
		Circle robot_circle;
		m_robot.computeTightBoundingCircle(&robot_circle);
		m_rvo_simulator.addAgent(toRVO(robot_circle.center), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon,
			robot_circle.radius + m_orca_distance_margin/2.f, m_robot.rds_configuration.vw_diamond_limits.v_max,
			toRVO(m_robot.last_step_p_ref_velocity));
	}

	m_rvo_simulator.setTimeStep(dt);
	m_rvo_simulator.doStep();

	if (!m_ignore_orca_circle && !m_orca_orca && m_robot.ORCA_implementation && m_robot.ORCA_use_p_ref)
		m_rvo_simulator.popBackAgent();

	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_pedestrians[i].velocity = toRDS(m_rvo_simulator.getAgentVelocity(i + offset + m_bounding_circles_robot.circles().size()));

	if ((!m_orca_orca) && m_robot_avoids)
	{
		for (const auto& sc : m_static_obstacles)
			m_pedestrians.push_back(MovingCircle(sc.circle, Vec2(0.f, 0.f)));
		
		m_robot.stepEuler(dt, getRobotNominalVelocity(), m_pedestrians);
		
		for (const auto& sc : m_static_obstacles)
			m_pedestrians.pop_back();
	}
	else
	{
		// update robot using RVO velocity
		unsigned int i = m_bounding_circles_robot.circles().size();
		Vec2 orca_velocity_p_ref = toRDS(m_rvo_simulator.getAgentVelocity(i));
		if ((!m_orca_orca))
			orca_velocity_p_ref = getRobotNominalVelocity();
		Vec2 v_global, v_for_angular_global, v_for_angular_local;
		m_robot.transformReferencePointVelocityToPointVelocity(m_robot.rds_configuration.p_ref,
			orca_velocity_p_ref, &v_global);
		m_robot.transformReferencePointVelocityToPointVelocity(Vec2(0.f, 1.f),
			orca_velocity_p_ref, &v_for_angular_global);
		m_robot.transformVectorGlobalToLocal(v_for_angular_global, &v_for_angular_local);
		float omega = -v_for_angular_local.x;

		m_robot.position = m_robot.position + dt*v_global;
		m_robot.orientation += dt*omega;
		m_robot.last_step_p_ref_velocity = orca_velocity_p_ref;
	}

	if (m_orca_orca)
	{
		if (false)
		{
			// correct any mismatch due to round off errors
			unsigned int i = m_bounding_circles_robot.circles().size();
			Vec2 v_global;
			// set RVO agent's position
			m_robot.transformVectorLocalToGlobal(m_robot.rds_configuration.p_ref, &v_global);
			m_rvo_simulator.setAgentPosition(i, toRVO(m_robot.position + v_global));
		}
		else
		{
			// do the reverse
			Vec2 rob_pos = toRDS(m_rvo_simulator.getAgentPosition(m_bounding_circles_robot.circles().size()));
			Vec2 v_global;
			m_robot.transformVectorLocalToGlobal(m_robot.rds_configuration.p_ref, &v_global);
			m_robot.position = rob_pos - v_global;
		}
	}

	for (std::vector<Circle>::size_type i = 0; i < m_bounding_circles_robot.circles().size(); i++)
	{
		const Vec2& p_local(m_bounding_circles_robot.circles()[i].center);
		Vec2 v_global;
		// set RVO agent's position
		m_robot.transformVectorLocalToGlobal(p_local, &v_global);
		Vec2 p_global(v_global + m_robot.position);
		m_rvo_simulator.setAgentPosition(i, toRVO(p_global));
		// set RVO agent's velocity
		m_robot.transformReferencePointVelocityToPointVelocity(p_local, m_robot.last_step_p_ref_velocity, &v_global);
		m_rvo_simulator.setAgentVelocity(i, toRVO(v_global));
	}

	/*Vec2 v_pos_to_p_ref_global;
	m_robot.transformVectorLocalToGlobal(m_robot.rds_configuration.p_ref, &v_pos_to_p_ref_global);
	m_rvo_simulator.setAgentPosition(0, toRVO(m_robot.position + v_pos_to_p_ref_global));
	m_rvo_simulator.setAgentVelocity(0, toRVO(m_robot.last_step_p_ref_velocity));*/

	m_time += dt;
}

Vec2 RdsOrcaSimulator::getRobotNominalVelocity()
{
	return Vec2(1.f, 0.f);
}

RVO::Vector2  RdsOrcaSimulator::getPedestrianNominalVelocity(unsigned int i)
{
	return RVO::Vector2(1.3f, 0.f);
}

void RdsOrcaSimulator::checkRobotCollisions()
{
	int offset = 0;
	if (m_orca_orca)
		offset = 1;
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
	{
		unsigned int ped_index = i + offset + m_bounding_circles_robot.circles().size();
		Vec2 ped_pos = toRDS(m_rvo_simulator.getAgentPosition(ped_index));
		float ped_radius = m_rvo_simulator.getAgentRadius(ped_index);
		
		Vec2 ped_pos_local;
		m_robot.transformVectorGlobalToLocal(ped_pos - m_robot.position, &ped_pos_local);
		const Geometry2D::Capsule& robot_shape(m_robot.rds_configuration.robot_shape);
		Vec2 pt_segment;
		robot_shape.closestMidLineSegmentPoint(ped_pos_local, &pt_segment);

		float delta = m_robot.rds_configuration.delta;
		float radius_sum = ped_radius - delta/2.f + robot_shape.radius(); //assumes orca margin = rds margin
		if ((pt_segment - ped_pos_local).norm() < radius_sum)// + delta)
			m_robot_collisions[i] = true;

		/*if (m_orca_orca)
		{
			unsigned int rob_index = m_bounding_circles_robot.circles().size();
			Vec2 rob_pos = toRDS(m_rvo_simulator.getAgentPosition(rob_index));
			float rob_radius = m_rvo_simulator.getAgentRadius(rob_index);
			if ((rob_pos - ped_pos).norm() < rob_radius + ped_radius)
				m_robot_collisions[i] = true;
		}
		else
		{
			for (unsigned int j = 0; j < m_bounding_circles_robot.circles().size(); j++)
			{
				Vec2 rob_pos = toRDS(m_rvo_simulator.getAgentPosition(j));
				float rob_radius = m_rvo_simulator.getAgentRadius(j);
				if ((rob_pos - ped_pos).norm() < rob_radius + ped_radius)
					m_robot_collisions[i] = true;
			}
		}*/
	}

	for (auto& sc : m_static_obstacles)
	{	
		Vec2 sc_pos_local;
		m_robot.transformVectorGlobalToLocal(sc.circle.center - m_robot.position, &sc_pos_local);
		const Geometry2D::Capsule& robot_shape(m_robot.rds_configuration.robot_shape);
		Vec2 pt_segment;
		robot_shape.closestMidLineSegmentPoint(sc_pos_local, &pt_segment);

		float orca_additional_margin = 0.03f + m_robot.rds_configuration.delta/2.f;
		if (!m_orca_orca)
			orca_additional_margin = 0.f;
		float delta = m_robot.rds_configuration.delta;
		float radius_sum = sc.circle.radius + robot_shape.radius();
		if ((pt_segment - sc_pos_local).norm() < radius_sum - orca_additional_margin)// + delta)
			sc.robot_collision = true;
	}
}

void RdsOrcaSimulator::addStaticObstacle(const Vec2& position, float radius, bool skip_rvo_simulator)
{
	m_static_obstacles.push_back(StaticCircle(position, radius, 0.03f));
	if (!skip_rvo_simulator)
	{
		m_rvo_simulator.addObstacle(m_static_obstacles.back().rvo_polygon);
		m_rvo_simulator.processObstacles();
	}
}


Vec2 init_robot_position(const RDS5CapsuleConfiguration& config,
	const CrowdTrajectory& crowd_trajectory, unsigned int robot_leader_index)
{
	Vec2 position;
	crowd_trajectory.getPedestrianPositionAtTime(robot_leader_index, 0.f, &position);
	Vec2 velocity;
	crowd_trajectory.getPedestrianVelocityAtTime(robot_leader_index, 0.f, &velocity);
	if (velocity.norm() == 0.f)
		return position - config.p_ref;
	else
	{
		float phi = std::atan2(velocity.y, velocity.x) - M_PI/2.f;
		Vec2 rotated_p_ref(std::cos(phi)*config.p_ref.x - std::sin(phi)*config.p_ref.y,
			std::sin(phi)*config.p_ref.x + std::cos(phi)*config.p_ref.y);
		return position - rotated_p_ref;
	}
}

float init_robot_orientation(const CrowdTrajectory& crowd_trajectory, unsigned int robot_leader_index)
{
	Vec2 velocity;
	crowd_trajectory.getPedestrianVelocityAtTime(robot_leader_index, 0.f, &velocity);
	if (velocity.norm() == 0.f)
		return 0.f;
	return std::atan2(velocity.y, velocity.x) - M_PI/2.f;
}

Vec2 init_robot_velocity(const RDS5CapsuleConfiguration& config,
	const CrowdTrajectory& crowd_trajectory, unsigned int robot_leader_index)
{
	Vec2 velocity;
	crowd_trajectory.getPedestrianVelocityAtTime(robot_leader_index, 0.f, &velocity);
	return velocity;
}

CrowdRdsOrcaSimulator::CrowdRdsOrcaSimulator(const RDS5CapsuleConfiguration& config,
	const CrowdTrajectory& crowd_trajectory, unsigned int robot_leader_index,
	 bool orca_orca)
	: RdsOrcaSimulator(init_robot_position(config, crowd_trajectory, robot_leader_index),
		init_robot_orientation(crowd_trajectory, robot_leader_index),
		config,
		init_robot_velocity(config, crowd_trajectory, robot_leader_index), orca_orca)
	, m_crowd_trajectory(crowd_trajectory)
	, m_robot_leader_index(robot_leader_index)
{ }

void CrowdRdsOrcaSimulator::addPedestrian(unsigned int crowd_pedestrian_index)
{
	Vec2 position, velocity;
	m_crowd_trajectory.getPedestrianPositionAtTime(crowd_pedestrian_index, 0.f, &position);
	m_crowd_trajectory.getPedestrianVelocityAtTime(crowd_pedestrian_index, 0.f, &velocity);
	unsigned int agent_no = RdsOrcaSimulator::addPedestrian(position, velocity);
	m_crowd_pedestrian_indices.push_back(crowd_pedestrian_index);
	m_pedestrians_avoidance.push_back(true);
	m_pedestrians_orca_no.push_back(agent_no);
	updateIgnoreInformation();
}

void CrowdRdsOrcaSimulator::disableAvoidanceForRecentlyAddedPedestrian()
{
	m_pedestrians_avoidance.back() = false;
	updateIgnoreInformation();
}

void CrowdRdsOrcaSimulator::updateIgnoreInformation()
{
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
	{
		if (!m_pedestrians_avoidance[i])
		{
			m_rvo_simulator.setAgentIgnoreAllIDs(m_pedestrians_orca_no[i]);
			std::vector<long unsigned int> ig_ids = m_rvo_simulator.getAgentIgnoreIDs(m_pedestrians_orca_no[i]);
			if (m_robot.ORCA_implementation)
				ig_ids.push_back(ig_ids.back() + 1);
			m_rvo_simulator.setAgentIgnoreIDs(m_pedestrians_orca_no[i], ig_ids);
			for (const auto& ig_id : ig_ids)
				std::cout << ig_id << " ";
			std::cout << std::endl;
			std::cout <<  m_rvo_simulator.getAgentID(m_pedestrians_orca_no[i]) << std::endl;
		}
	}
}

RVO::Vector2 CrowdRdsOrcaSimulator::getPedestrianNominalVelocity(unsigned int i)
{
	unsigned int crowd_pedestrian_index = m_crowd_pedestrian_indices[i];
	Vec2 feed_forward_velocity, position;
	m_crowd_trajectory.getPedestrianVelocityAtTime(crowd_pedestrian_index, m_time,
		&feed_forward_velocity);
	m_crowd_trajectory.getPedestrianPositionAtTime(crowd_pedestrian_index, m_time,
		&position);
	Vec2 feed_back_velocity(0.5f*(position - m_pedestrians[i].circle.center));
	return toRVO(feed_forward_velocity + feed_back_velocity);
}

Vec2 CrowdRdsOrcaSimulator::getPedestrianNominalPosition(unsigned int i) const
{
	unsigned int crowd_pedestrian_index = m_crowd_pedestrian_indices[i];
	Vec2 position;
	m_crowd_trajectory.getPedestrianPositionAtTime(crowd_pedestrian_index, m_time,
		&position);
	return position;
}

Vec2 CrowdRdsOrcaSimulator::getRobotNominalVelocity()
{
	Vec2 feed_forward_velocity, position;
	m_crowd_trajectory.getPedestrianVelocityAtTime(m_robot_leader_index, m_time,
		&feed_forward_velocity);
	m_crowd_trajectory.getPedestrianPositionAtTime(m_robot_leader_index, m_time,
		&position);
	Vec2 v_result;
	m_robot.transformVectorLocalToGlobal(m_robot.rds_configuration.p_ref, &v_result);
	Vec2 robot_p_ref_position(m_robot.position + v_result);
	Vec2 feed_back_velocity(0.25f*(position - robot_p_ref_position));
	//Vec2 disturbance = 0.5f*std::sin(m_time*6.f*2.f*M_PI)*Vec2(-feed_forward_velocity.y, feed_forward_velocity.x);
	
	Vec2 nominal_v = feed_forward_velocity + feed_back_velocity;

	RDS5CapsuleConfiguration& config = m_robot.rds_configuration;
	float v_max = std::max(config.vw_diamond_limits.v_max,
			std::sqrt(config.vw_diamond_limits.w_abs_max*config.p_ref.y*config.vw_diamond_limits.w_abs_max*config.p_ref.y +
			config.vw_diamond_limits.v_at_w_abs_max*config.vw_diamond_limits.v_at_w_abs_max));
	if (nominal_v.norm() > std::abs(v_max))
		nominal_v = nominal_v.normalized()*std::abs(v_max);

	return nominal_v;// + disturbance;
}

Circle RdsOrcaSimulator::getOrcaOrcaCircle() const
{
	if (!m_orca_orca)
		return Circle(Vec2(), 0.f);
	unsigned int i = m_bounding_circles_robot.circles().size();
	return Circle(toRDS(m_rvo_simulator.getAgentPosition(i)), m_rvo_simulator.getAgentRadius(i));
}

//setAgentIgnoreIDs(size_t agentNo, const std::vector<size_t>& ignore_ids)

//setAgentDenyCollisions(size_t agentNo, bool deny_collisions)