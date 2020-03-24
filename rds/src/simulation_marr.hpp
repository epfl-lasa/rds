#ifndef SIMULATION_MARR_HPP
#define SIMULATION_MARR_HPP

// simulation MARR (multiple agents reactive robot)

#include "geometry.hpp"
#include "rds_4_agent.hpp"
#include <RVO.h> // external official RVO2 library
#include <vector>


//struct StateMARR
//{
//	StateMARR(RVO::Simulator& rvo_sim) : rvo_sim(rvo_sim) { }

//private:
//	RVO::Simulator& rvo_sim;
//};

/*

d v_orca /dt = f(x, v_orca)
d x_rds /dt = g(x, v_orca)

*/


struct AgentMARR
{
	AgentMARR(int id, const Geometry2D::Vec2& position, double orientation, double radius, double v_max)
	: m_id(id), m_position(position), m_orientation(orientation), m_radius(radius), m_v_max(v_max) { }

	virtual ~AgentMARR() { }

	virtual void computeVelocity(double time, const std::vector<AgentMARR*>& agents,
		const AgentMARR* robot) = 0;

	void stepEuler(double dt) { m_position += dt*m_cartesian_velocity; m_orientation += dt*m_angular_velocity; };

	int getID() { return m_id; }
	void getPosition(Geometry2D::Vec2* result) { *result = m_position; }
	double getOrientation() { return m_orientation; }
	void getCartesianVelocity(Geometry2D::Vec2* result) { *result = m_cartesian_velocity; }
	double getAngularVelocity() { return m_angular_velocity; }

	virtual void getPositionAndPreferredVelocity(double time, const Geometry2D::Vec2& client_position,
		Geometry2D::Vec2* position_result, Geometry2D::Vec2* velocity_result) const = 0;

	int m_id;
	Geometry2D::Vec2 m_position, m_cartesian_velocity;
	double m_orientation, m_angular_velocity, m_radius, m_v_max;
};

//struct RobotMARR
//{
//	void stepEuler(double dt) = 0;
//};

struct SimulationMARR
{
	SimulationMARR() : m_time(0.0), m_robot(0) { }

	void stepEuler(double dt);

	std::vector<AgentMARR*> m_agents;
	AgentMARR* m_robot;
	double m_time;
	RVO::RVOSimulator m_reference_rvo_simulator;
};

typedef void (*IndexedVectorField)(int index, double time, const Geometry2D::Vec2& position, const AgentMARR* a,
	Geometry2D::Vec2* velocity_result);

struct RVOAgentMARR : public AgentMARR
{
	RVOAgentMARR(int id, const Geometry2D::Vec2& position, double orientation, double radius, double v_max, IndexedVectorField motion_law)
	: AgentMARR(id, position, orientation, radius, v_max), m_tau(1.f), m_delta(0.05f), m_motion_law(motion_law) { }

	virtual void computeVelocity(double time, const std::vector<AgentMARR*>& agents,
		const AgentMARR* robot);

	virtual void getPositionAndPreferredVelocity(double time, const Geometry2D::Vec2& client_position,
		Geometry2D::Vec2* position_result, Geometry2D::Vec2* velocity_result) const;

	RVO::RVOSimulator m_rvo_simulator;
	float m_tau, m_delta;
	IndexedVectorField m_motion_law;
};

struct RDSAgentMARR : public AgentMARR
{
	RDSAgentMARR(int id, const Geometry2D::Vec2& position, double orientation, double radius, double v_max)
	: AgentMARR(id, position, orientation, radius, v_max) { }

	virtual void computeVelocity(double time, const std::vector<AgentMARR*>& agents,
		const AgentMARR* robot);

	virtual void getPositionAndPreferredVelocity(double time, const Geometry2D::Vec2& client_position,
		Geometry2D::Vec2* position_result, Geometry2D::Vec2* velocity_result) const;
};

//struct RobotMARR
//{
//	void stepEuler(double dt) = 0;
//};

#endif