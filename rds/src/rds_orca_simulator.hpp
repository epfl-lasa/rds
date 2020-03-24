#ifndef RDS_ORCA_SIMULATOR
#define RDS_ORCA_SIMULATOR

#include "geometry.hpp"
#include "rds_4_agent.hpp"
#include <RVO.h>
#include <vector>

struct RdsOrcaSimulator
{
	RdsOrcaSimulator(const Geometry2D::Vec2& position, float orientation,
		const RDSCapsuleConfiguration& config, const Geometry2D::Vec2& reference_point_velocity);

	virtual ~RdsOrcaSimulator() { }

	void addPedestrian(const Geometry2D::Vec2& position, const Geometry2D::Vec2& velocity);

	void setRobotProperties(const Geometry2D::Vec2& position, float orientation,
		const RDSCapsuleConfiguration& config, const Geometry2D::Vec2& reference_point_velocity);

	void step(float dt);

	const std::vector<MovingCircle>& getPedestrians() const { return m_pedestrians; }

	const RDS4CapsuleAgent& getRobot() const { return m_robot; }
protected:
	Geometry2D::Vec2 getRobotNominalVelocity();

	RVO::Vector2 getPedestrianNominalVelocity(unsigned int i);
protected:
	RVO::RVOSimulator m_rvo_simulator;
	std::vector<MovingCircle> m_pedestrians;
	RDS4CapsuleAgent m_robot;
	float m_time;
public:
	const float m_orca_time_horizon;
	const float m_orca_distance_margin;
	const float m_pedestrian_radius;
	const float m_pedestrian_v_max;
};

#endif