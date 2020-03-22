#include "simulation_marr.hpp"
#include "gui.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <thread>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;

std::vector<Vec2> goals;

std::vector<Circle> circles_global;
std::vector<Circle> reference_circles_global;

std::chrono::milliseconds gui_cycle_time(50);
std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();

auto drive_to_indexed_goal = [&goals](int index, double time, const Vec2& position, const AgentMARR* a,
	Vec2* velocity_result)
{
	*velocity_result = a->m_v_max*(goals[index] - position)/std::min(1.f, (goals[index] - position).norm());
};

static Vec2 toVec2(const RVO::Vector2& v)
{
	return Vec2(v.x(), v.y());
}

bool update_gui(GUI* gui, const SimulationMARR& sim)
{
	for (unsigned int i = 0; i < circles_global.size(); i++)
	{
		circles_global[i].center = sim.m_agents[i]->m_position;
		reference_circles_global[i].center = toVec2(sim.m_reference_rvo_simulator.getAgentPosition(i));
	}

	for (auto& c : reference_circles_global)
		circles_global.push_back(c);

	std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - t_gui_update));
	t_gui_update = std::chrono::high_resolution_clock::now();

	bool still_open = (gui->update() == 0);

	for (auto& c : reference_circles_global)
		circles_global.pop_back();
	return still_open;
}

void simulate(SimulationMARR* sim, float window_size)
{
	GUI gui("Demo MARR", window_size);
	gui.circles = &circles_global;
	GuiColor red;
	red.g = red.b = 0;
	red.r = 255;
	for (auto& a : sim->m_agents)
	{
		circles_global.push_back(Circle(a->m_position, a->m_radius));
		gui.circles_colors.push_back(red);
		reference_circles_global.push_back(Circle(a->m_position, a->m_radius));
	}

	float dt = gui_cycle_time.count()*0.001f;
	double time = 0.0;
	do
	{
		sim->stepEuler(dt);
	}
	while (update_gui(&gui, *sim));
}

int main()
{
	SimulationMARR sim;
	int n_agents = 100;
	float agent_radius = 0.25f;
	float R = n_agents*agent_radius*3.f/M_PI/2.f;
	float v_max = 1.5f;
	for (int i = 0; i < n_agents; i++)
	{
		Vec2 position(R*std::cos(i*2.f*M_PI/n_agents), R*std::sin(i*2.f*M_PI/n_agents));
		sim.m_agents.push_back(new RVOAgentMARR(i, position, 0.f, agent_radius, v_max,
			drive_to_indexed_goal));
		goals.push_back(-1.f*Vec2(R*std::cos((i + n_agents/20)*2.f*M_PI/n_agents), R*std::sin((i + n_agents/20)*2.f*M_PI/n_agents)) + 0.5f*Vec2(std::cos(i), std::sin(i)));
	}

	simulate(&sim, R*2.2f);

	for (auto& a : sim.m_agents)
		delete a;

	return 0;
}