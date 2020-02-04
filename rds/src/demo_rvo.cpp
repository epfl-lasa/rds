#include "simulation.hpp"
#include "gui.hpp"
#include "geometry.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <chrono>
#include <thread>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;

void simulate_while_displaying(Simulation* s, const char* title = "RVO")
{
	GUI gui_constraints(title, 6.f);
	std::vector<HalfPlane2> ref_p_constraints;
	gui_constraints.halfplanes = &ref_p_constraints;

	GUI gui_work_space(title, 6.f);
	std::vector<Circle> work_space_circles;
	gui_work_space.circles = &work_space_circles;

	std::chrono::milliseconds gui_cycle_time(50);
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	while ((gui_work_space.update() == 0) | (gui_constraints.update() == 0))
	{
		for (int i = 0; i < 50; i++)
			s->stepEuler(0.001);
		// transfer the positions to the gui variables
		{
			work_space_circles.resize(0);
			for (auto& a : s->agents)
			{
				for (std::vector<Circle>::size_type i = 0; i != a->circles.size(); i++)
				{
					Circle c;
					Vec2 v;
					a->getCircleAndNominalVelocityGlobal(i, 0.f, &c, &v);
					work_space_circles.push_back(c);
				}
			}

			ref_p_constraints.resize(0);
			if (s->agents.size() > 0)
			{
				for(auto& h : s->agents[0]->constraints)
					ref_p_constraints.push_back(h);
			}
		}

		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}
}

int main(int argc, char** argv)
{
	Agent a1, a2, a3;
	DifferentialDriveAgent d;
	Simulation s(RVO(5.f, 0.05f));
	s.agents.push_back(&d);
	s.agents.push_back(&a1);
	//s.agents.push_back(&a2);
	//s.agents.push_back(&a3);

	a1.position = Vec2(-1.5f, 0.f);
	a2.position = Vec2(0.f, -1.f);
	a3.position = Vec2(2.5f, 1.75f);
	d.position = Vec2(0.f, -1.5f);
	d.orientation = 0.f;

	a1.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
	a2.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
	/*a2.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.25));
	a2.circles.push_back(Circle(Vec2(0.f, -0.4f), 0.25));
	a2.circles.push_back(Circle(Vec2(0.f, -0.6f), 0.25));
	a2.circles.push_back(Circle(Vec2(-0.075f, -0.4f), 0.25));
	//a2.circles.push_back(Circle(Vec2(-0.2f, -0.3f), 0.25));
	a2.circles.push_back(Circle(Vec2(-0.15f, -0.1f), 0.25));
	a2.circles.push_back(Circle(Vec2(0.075f, -0.4f), 0.25));
	//a2.circles.push_back(Circle(Vec2(0.2f, -0.3f), 0.25));
	a2.circles.push_back(Circle(Vec2(0.15f, -0.1f), 0.25));*/
	a2.circles.push_back(Circle(Vec2(0.f, -0.35f), 0.2));

	a3.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));

	d.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
	d.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));

	Environment e1, e2, e3;
	e1.orientation = 0.f;
	e1.speed = 0.2f;
	e2.orientation = M_PI/2.f;
	e2.speed = 0.2f;
	e3.orientation = M_PI;
	e3.speed = 0.2f*0.0;;
	a1.environment = &e1;
	a2.environment = &e2;
	a3.environment = &e3;
	d.environment = &e2;

	simulate_while_displaying(&s);
}

