#include "cvo.hpp"
#include "gui.hpp"

#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include <iostream>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;
using Geometry2D::CVO;

int main()
{
	GUI gui_objects("Objects", 3.f);

	std::vector<Circle> work_space_circles;
	gui_objects.circles = &work_space_circles;

	GUI gui_constraints("CVO construction", 6.f);

	std::vector<HalfPlane2> constraints;
	gui_constraints.halfplanes = &constraints;
	std::vector<Arrow> arrows;
	gui_constraints.arrows = &arrows;
	std::vector<Circle> circles;
	gui_constraints.circles = &circles;

	float theta = 1.f;

	while ((gui_objects.update() == 0) | (gui_constraints.update() == 0))
	{
		work_space_circles.resize(0);
		constraints.resize(0);
		arrows.resize(0);
		circles.resize(0);

		Circle o1(Vec2(0.0f, 0.f), 0.2f);
		Circle o2(Vec2(0.f, 0.9f), 0.2f);
		work_space_circles.push_back(o1);
		work_space_circles.push_back(o2);

		theta += 0.005f;
		Vec2 v_o1_opt = Vec2(std::cos(theta), std::sin(theta));
		v_o1_opt = Vec2(0.f, 1.f);

		Vec2 v_o2 = Vec2(std::cos(2.324*theta), std::sin(2.324*theta));
		v_o2 = Vec2(-0.25f, 0.f);

		CVO cvo(o1.center, o2.center, v_o1_opt, v_o2, o1.radius + o2.radius, 1.f); //tau

		constraints.push_back(cvo.getConservativeConstraint());
		constraints.back().shift(-1.f*v_o2);
		constraints.push_back(cvo.getRelativeVOHPlus());
		//constraints.back().shift(v_o2);
		constraints.push_back(cvo.getRelativeVOHMinus());
		//constraints.back().shift(v_o2);
		circles.push_back(Circle(cvo.getRelativeVOCenter(),// + v_o2,
			cvo.getVORadius()));

		arrows.push_back(Arrow(v_o1_opt - v_o2, Vec2(0.f, 0.f)));
		arrows.push_back(Arrow(-1.f*v_o2, Vec2(0.f, 0.f)));

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
}