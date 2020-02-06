#include "RVO.hpp"
#include "gui.hpp"

#include <vector>
#include <chrono>
#include <thread>

#include <iostream>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;

int main()
{
	GUI gui_objects("Objects", 3.f);
	Circle o1(Vec2(-0.5f, -1.f), 0.5f);
	Circle o2(Vec2(-0.8f, 0.5f), 0.5f);

	std::vector<Circle> work_space_circles;
	gui_objects.circles = &work_space_circles;
	work_space_circles.push_back(o1);
	work_space_circles.push_back(o2);

	GUI gui_constraints("RVO construction", 6.f);
	std::vector<HalfPlane2> constraints;
	gui_constraints.halfplanes = &constraints;
	std::vector<Arrow> arrows;
	gui_constraints.arrows = &arrows;
	std::vector<Circle> circles;
	gui_constraints.circles = &circles;

	Vec2 vpref_1(0.05f, 0.05f);
	Vec2 vpref_2(-0.f, -0.05f);
	HalfPlane2 vo_1, vo_2;

	RVO rvo(1.f, 0.f);
	rvo.computeCoordinativeVelocityObstacles(o1, o2, vpref_1, vpref_2, &vo_1, &vo_2);

	arrows.push_back(Arrow(Vec2(0.f, 0.f), -1.f*vpref_2));
	arrows.push_back(Arrow(vpref_1 - vpref_2, Vec2(0.f, 0.f)));
	arrows.push_back(Arrow(vpref_1 - vpref_2, -1.f*vpref_2));

	Vec2 relative_position(o2.center - o1.center);
	float v_c = (relative_position.norm() - (o1.radius + o2.radius + rvo.getDelta()))/rvo.getTau();
	float v_r = (o1.radius + o2.radius + rvo.getDelta())/rvo.getTau();
	circles.push_back(Circle((v_c + v_r)*relative_position.normalized(), v_r));

	// compute the tightest pair of halfplanes whose intersection contains the RVO-cone
	float r_thales_circle = 0.5f*(v_c + v_r);
	float tangent_point_1st_coord = r_thales_circle - v_r*v_r/2.f/r_thales_circle;
	float tangent_point_2nd_coord_squared = r_thales_circle*r_thales_circle - 
		tangent_point_1st_coord*tangent_point_1st_coord;

	Vec2 e_basis_1(relative_position.normalized());
	Vec2 e_basis_2(-e_basis_1.y, e_basis_1.x);
	Vec2 c_thales_circle(relative_position.normalized()*r_thales_circle);
	Vec2 p_tangent_plus(c_thales_circle + e_basis_1*tangent_point_1st_coord +
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));
	Vec2 p_tangent_minus(c_thales_circle + e_basis_1*tangent_point_1st_coord -
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));

	Vec2 c_cone_cap(relative_position.normalized()*(v_c + v_r));

	HalfPlane2 h_minus(p_tangent_minus - c_cone_cap, 0.f);
	HalfPlane2 h_plus(p_tangent_plus - c_cone_cap, 0.f);

	//constraints.push_back(h_minus);
	//constraints.push_back(h_plus);
	constraints.push_back(rvo.m_convex_rvo);

	while ((gui_objects.update() == 0) | (gui_constraints.update() == 0))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
}