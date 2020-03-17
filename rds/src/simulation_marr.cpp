#include "simulation_marr.hpp"

using Geometry2D::Vec2;

RVO::Vector2 toRVO(const Vec2& v)
{
	return RVO::Vector2(v.x, v.y);
}

Vec2 toVec2(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

void SimulationMARR::stepEuler(double dt)
{	
	// reference simulation
	if (m_reference_rvo_simulator.getNumAgents() == 0)
	{
		for (auto& a : m_agents)
		{
			RVOAgentMARR* b = dynamic_cast<RVOAgentMARR*>(a);
			m_reference_rvo_simulator.addAgent(toRVO(a->m_position), 15.0f, 10, b->m_tau, b->m_tau,
				a->m_radius + b->m_delta/2.f, a->m_v_max);
		}
	}
	for (unsigned int i = 0; i != m_reference_rvo_simulator.getNumAgents(); ++i)
	{
		Vec2 velocity_result;
		dynamic_cast<RVOAgentMARR*>(m_agents[i])->m_motion_law(i, m_time, toVec2(
			m_reference_rvo_simulator.getAgentPosition(i)), m_agents[i], &velocity_result);
		m_reference_rvo_simulator.setAgentPrefVelocity(i, toRVO(velocity_result));
	}
	m_reference_rvo_simulator.setTimeStep(dt);
	m_reference_rvo_simulator.doStep();

	// actual simulation

	for (auto& a : m_agents)
		a->computeVelocity(m_time, m_agents, m_robot);
	if (m_robot)
		m_robot->computeVelocity(m_time, m_agents, 0);

	for (auto& a : m_agents)
		a->stepEuler(dt);
	if (m_robot)
		m_robot->stepEuler(dt);
	m_time += dt;
}

void RVOAgentMARR::computeVelocity(double time, const std::vector<AgentMARR*>& agents, const AgentMARR* robot)
{
	RVO::RVOSimulator rvo_simulator;
	rvo_simulator.setTimeStep(1.f);
	if (rvo_simulator.getNumAgents() == 0)
	{
		RVO::Vector2 p_zero(0.f, 0.f);
		rvo_simulator.addAgent(p_zero, 15.0f, 10, m_tau, m_tau, m_radius, m_v_max);
		if (robot)
			rvo_simulator.addAgent(p_zero, 15.0f, 10, m_tau, m_tau, robot->m_radius + m_delta, m_v_max);

		for (auto& a : agents)
		{
			if (a->m_id != m_id)
				rvo_simulator.addAgent(p_zero, 15.0f, 10, m_tau, m_tau, a->m_radius + m_delta, m_v_max);
		}
	}

	rvo_simulator.setAgentPosition(0, toRVO(m_position));
	Vec2 position_result, velocity_result;
	this->getPositionAndPreferredVelocity(time, m_position, &position_result, &velocity_result);
	rvo_simulator.setAgentPrefVelocity(0, toRVO(velocity_result));

	if (robot)
	{
		robot->getPositionAndPreferredVelocity(time, m_position, &position_result, &velocity_result);
		rvo_simulator.setAgentPosition(1, toRVO(position_result));
		rvo_simulator.setAgentPrefVelocity(1, toRVO(velocity_result));
	}
	unsigned int i = 1;
	if (robot)
		i++;
	for (auto& a : agents)
	{
		if (a->m_id != m_id)
		{
			a->getPositionAndPreferredVelocity(time, m_position, &position_result, &velocity_result);
			rvo_simulator.setAgentPosition(i, toRVO(position_result));
			rvo_simulator.setAgentPrefVelocity(i, toRVO(velocity_result));
			i++;
		}
	}

	m_cartesian_velocity = toVec2(rvo_simulator.computeStepForSingleAgent(0));
	m_angular_velocity = 0.0;
}

void RVOAgentMARR::getPositionAndPreferredVelocity(double time, const Geometry2D::Vec2& client_position,
	Geometry2D::Vec2* position_result, Geometry2D::Vec2* velocity_result) const
{
	*position_result = m_position;
	if (!m_motion_law)
		*velocity_result = Vec2(0.f, 0.f);
	else
		(*m_motion_law)(m_id, time, m_position, this, velocity_result);
}