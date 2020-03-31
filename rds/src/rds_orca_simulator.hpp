#ifndef RDS_ORCA_SIMULATOR
#define RDS_ORCA_SIMULATOR

#include "geometry.hpp"
#include "rds_4_agent.hpp"
#include "crowd_trajectory.hpp"
#include <RVO.h>
#include <vector>

struct RdsOrcaSimulator
{
	RdsOrcaSimulator(const Geometry2D::Vec2& position, float orientation,
		const RDSCapsuleConfiguration& config, const Geometry2D::Vec2& reference_point_velocity, bool orca_orca = false);

	virtual ~RdsOrcaSimulator() { }

	void addPedestrian(const Geometry2D::Vec2& position, const Geometry2D::Vec2& velocity);

	void setRobotProperties(const Geometry2D::Vec2& position, float orientation,
		const RDSCapsuleConfiguration& config, const Geometry2D::Vec2& reference_point_velocity);

	void step(float dt);

	const std::vector<MovingCircle>& getPedestrians() const { return m_pedestrians; }

	const RDS4CapsuleAgent& getRobot() const { return m_robot; }

	const Geometry2D::BoundingCircles& getBoundingCirclesRobot() const { return m_bounding_circles_robot; }

	float getTime() const { return m_time; }

	AdditionalPrimitives2D::Circle getOrcaOrcaCircle() const;
protected:
	virtual Geometry2D::Vec2 getRobotNominalVelocity();

	virtual RVO::Vector2 getPedestrianNominalVelocity(unsigned int i);
protected:
	RVO::RVOSimulator m_rvo_simulator;
	std::vector<MovingCircle> m_pedestrians;
	RDS4CapsuleAgent m_robot;
	Geometry2D::BoundingCircles m_bounding_circles_robot;
	float m_time;
public:
	const float m_orca_time_horizon;
	const float m_orca_distance_margin;
	const float m_pedestrian_radius;
	const float m_pedestrian_v_max;
	const bool m_orca_orca;
};


struct CurveRdsOrcaSimulator : public RdsOrcaSimulator
{
	CurveRdsOrcaSimulator(const Geometry2D::Vec2& position, float orientation,
		const RDSCapsuleConfiguration& config, const Geometry2D::Vec2& reference_point_velocity);
protected:
	virtual Geometry2D::Vec2 getRobotNominalVelocity();

	virtual RVO::Vector2 getPedestrianNominalVelocity(unsigned int i);

	Geometry2D::Vec2 getVortexVelocity(const Geometry2D::Vec2& position);

	Geometry2D::Vec2 m_vortex_center;
	float m_omega;
};

struct CrowdRdsOrcaSimulator : public RdsOrcaSimulator
{
	CrowdRdsOrcaSimulator(const RDSCapsuleConfiguration& config,
		const CrowdTrajectory& crowd_trajectory, unsigned int robot_leader_index,
		bool orca_orca = false);

	const CrowdTrajectory m_crowd_trajectory;

	void addPedestrian(unsigned int crowd_pedestrian_index);

	Geometry2D::Vec2 getPedestrianNominalPosition(unsigned int i) const;

	const unsigned int m_robot_leader_index;
protected:
	virtual RVO::Vector2 getPedestrianNominalVelocity(unsigned int i);

	virtual Geometry2D::Vec2 getRobotNominalVelocity();

	std::vector<unsigned int> m_crowd_pedestrian_indices;
};

/*struct CrowdOrcaOrcaSimulator
{
	CrowdOrcaOrcaSimulator(const CrowdRdsOrcaSimulator& ref_sim);
};*/

#endif