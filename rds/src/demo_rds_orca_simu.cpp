#include "gui.hpp"
#include "rds_orca_simulator.hpp"
#include <chrono>
#include <thread>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;
using Geometry2D::BoundingCircles;

struct GuiWrap
{
	GuiWrap(const RdsOrcaSimulator& sim)
	: m_gui("RDS-ORCA Simulator", 20.f)
	, m_bounding_circles(sim.getBoundingCirclesRobot())
	{
		m_gui.circles = &m_circles;
		m_gui.capsules = &m_capsules;
		for (auto& p : sim.getPedestrians())
			m_circles.push_back(p.circle);
		m_capsules.push_back(sim.getRobot().rds_configuration.robot_shape);
		for (auto& c : m_bounding_circles.circles())
			m_circles.push_back(c);
	}

	bool update(const RdsOrcaSimulator& sim)
	{
		for (std::vector<Circle>::size_type i = 0; i < m_circles.size() - m_bounding_circles.circles().size(); i++)
			m_circles[i].center = sim.getPedestrians()[i].circle.center;
		const Capsule& robot_shape(sim.getRobot().rds_configuration.robot_shape);
		Vec2 v_result;
		sim.getRobot().transformVectorLocalToGlobal(robot_shape.center_a(), &v_result);
		Vec2 center_a = v_result + sim.getRobot().position;
		sim.getRobot().transformVectorLocalToGlobal(robot_shape.center_b(), &v_result);
		Vec2 center_b = v_result + sim.getRobot().position;
		m_capsules[0] = Capsule(m_capsules[0].radius(), center_a, center_b);

		for (std::vector<Circle>::size_type i = m_circles.size() - m_bounding_circles.circles().size(); i < m_circles.size(); i++)
		{
			unsigned int bc_index = i - (m_circles.size() - m_bounding_circles.circles().size());
			sim.getRobot().transformVectorLocalToGlobal(m_bounding_circles.circles()[bc_index].center, &v_result);
			m_circles[i] = Circle(v_result + sim.getRobot().position, m_bounding_circles.circles()[bc_index].radius);
		}

		return (m_gui.update() == 0);
	}

	GUI m_gui;
	std::vector<Circle> m_circles;
	std::vector<Capsule> m_capsules;
	const BoundingCircles& m_bounding_circles;
};

int main()
{
	float y_ref = 0.2f;
	RDSCapsuleConfiguration config(1.f, 0.05f, 1.7f,
		Capsule(0.4f, Vec2(0.f, y_ref), Vec2(0.f, -0.3f)), Vec2(0.f, y_ref));
	CurveRdsOrcaSimulator sim(Vec2(0.f, 0.f), -3.141f/2.f, config, Vec2(1.f, 0.f));

	sim.addPedestrian(Vec2(2.f, 0.5f), Vec2(1.f, 0.f));
	sim.addPedestrian(Vec2(-2.f, 0.f), Vec2(1.f, 0.f));
	sim.addPedestrian(Vec2(-2.2f, 0.5f), Vec2(1.f, 0.f));
	sim.addPedestrian(Vec2(-2.5f, -0.5f), Vec2(1.f, 0.f));
	sim.addPedestrian(Vec2(1.5f, 0.2f), Vec2(1.f, 0.f));

	GuiWrap gui_wrap(sim);

	float dt = 0.02f;
	std::chrono::milliseconds gui_cycle_time(int(dt*1000.f));
	std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
	do
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
		sim.step(dt);
	}
	while (gui_wrap.update(sim));
}