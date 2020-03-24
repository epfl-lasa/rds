#include "rds_orca_simulator.hpp"
#include <cmath>
using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;

static RVO::Vector2 toRVO(const Vec2& v)
{
	return RVO::Vector2(v.x, v.y);
}

static Vec2 toRDS(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

RdsOrcaSimulator::RdsOrcaSimulator(const Vec2& position, float orientation,
	const RDSCapsuleConfiguration& config, const Vec2& reference_point_velocity)
	: m_time(0.f)
	, m_orca_time_horizon(1.f)
	, m_orca_distance_margin(0.05f)
	, m_pedestrian_radius(0.25f)
	, m_pedestrian_v_max(1.8f)
{
	m_rvo_simulator.addAgent(RVO::Vector2(0.f, 0.f), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon, 1.f, 1.f);
	setRobotProperties(position, orientation, config, reference_point_velocity);
}

void RdsOrcaSimulator::addPedestrian(const Vec2& position, const Vec2& velocity)
{
	m_rvo_simulator.addAgent(toRVO(position), 15.0f, 10, m_orca_time_horizon, m_orca_time_horizon,
		m_pedestrian_radius + m_orca_distance_margin/2.f, m_pedestrian_v_max, toRVO(velocity));
	m_pedestrians.push_back(MovingCircle(Circle(position, m_pedestrian_radius), Vec2()));
}

void RdsOrcaSimulator::setRobotProperties(const Vec2& position, float orientation,
	const RDSCapsuleConfiguration& config, const Vec2& reference_point_velocity)
{
	m_robot = RDS4CapsuleAgent(position, orientation, config);

	Vec2 v_pos_to_p_ref_global;
	m_robot.transformVectorLocalToGlobal(config.p_ref, &v_pos_to_p_ref_global);
	m_rvo_simulator.setAgentPosition(0, toRVO(position + v_pos_to_p_ref_global));
	float radius = config.robot_shape.radius + std::max(
		(config.robot_shape.center_a - config.p_ref).norm(),
		(config.robot_shape.center_b - config.p_ref).norm());
	m_rvo_simulator.setAgentRadius(0, radius + m_orca_distance_margin/2.f);
	m_rvo_simulator.setAgentVelocity(0, toRVO(reference_point_velocity));
	m_rvo_simulator.setAgentMaxSpeed(0, config.v_max);
}

void RdsOrcaSimulator::step(float dt)
{
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_pedestrians[i].circle.center = toRDS(m_rvo_simulator.getAgentPosition(i + 1));

	m_rvo_simulator.setAgentPrefVelocity(0, toRVO(getRobotNominalVelocity()));
	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_rvo_simulator.setAgentPrefVelocity(i + 1, getPedestrianNominalVelocity(i));

	m_rvo_simulator.setTimeStep(dt);
	m_rvo_simulator.doStep();

	for (unsigned int i = 0; i < m_pedestrians.size(); i++)
		m_pedestrians[i].velocity = toRDS(m_rvo_simulator.getAgentVelocity(i + 1));

	m_robot.stepEuler(dt, getRobotNominalVelocity(), m_pedestrians);

	Vec2 v_pos_to_p_ref_global;
	m_robot.transformVectorLocalToGlobal(m_robot.rds_configuration.p_ref, &v_pos_to_p_ref_global);
	m_rvo_simulator.setAgentPosition(0, toRVO(m_robot.position + v_pos_to_p_ref_global));
	m_rvo_simulator.setAgentVelocity(0, toRVO(m_robot.last_step_p_ref_velocity));

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