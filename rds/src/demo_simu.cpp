#include "simulator.hpp"
#include "gui.hpp"

#include <chrono>
#include <thread>
//#define _USE_MATH_DEFINES
#include <cmath>

using AdditionalPrimitives2D::Circle;
using Geometry2D::Vec2;
using RDS::VelocityCommand;

int main()
{
	std::vector<Circle> robot_shape;
	robot_shape.push_back(Circle(Vec2(0.f, 0.056f), 0.4f));
	robot_shape.push_back(Circle(Vec2(0.f, -0.517), 0.3f));

	RDS::Simulator simu(
		[](float time, const Vec2& position, float orientation) {return VelocityCommand(0.5f, 0.f);}, // nominal control law
		robot_shape,
		Vec2(0.f, -3.f), // initial position
		0.f, // initial orientation
		VelocityCommand(0.f, 0.f)); // previous command

	simu.obstacles.push_back(RDS::Simulator::Obstacle(
		[](float time, const Vec2& position) {
			float t = time/6.f-0.8f;
			return Vec2(-0.6f*std::exp(-t*t)*t, 1.f);},
		Vec2(0.0f, -6.f),
		0.25f));

	/*simu.obstacles.push_back(RDS::Simulator::Obstacle(
		[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
		Vec2(1.f, 1.f),
		0.25f));

	simu.obstacles.push_back(RDS::Simulator::Obstacle(
		[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
		Vec2(1.f, 1.5f),
		0.25f));*/

	GUI gui_work_space("Robot Motion", 12.f);
	//std::vector<Geometry2D::Vec2> work_space_points;
	//std::vector<AdditionalPrimitives2D::Arrow> work_space_arrows;
	std::vector<AdditionalPrimitives2D::Circle> work_space_circles;
	//gui_work_space.points = &work_space_points;
	gui_work_space.circles = &work_space_circles;
	//gui_work_space.arrows = &work_space_arrows;

	GuiColor green, blue, red;
	green.r = green.b = 0;
	green.g = 255;
	blue.r = blue.g = 0;
	blue.b = 255;
	red.g = red.b = 0;
	red.r = 255;

	std::chrono::milliseconds gui_cycle_time(50);
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	while (gui_work_space.update() == 0)
	{
		simu.stepEuler(0.01);
		// transfer the positions to the gui variables
		{
			float rxx = std::cos(simu.robot.orientation);
			float rxy = std::sin(simu.robot.orientation);
			float ryx = -rxy;
			float ryy = rxx;

			work_space_circles.resize(0);
			for (auto& ob : simu.obstacles)
				work_space_circles.push_back(Circle(ob.position, ob.radius));

			for (auto& rs : simu.robot.shape)
			{
				Vec2 center_global = Vec2(rxx*rs.center.x + ryx*rs.center.y,
					rxy*rs.center.x + ryy*rs.center.y) + simu.robot.position;
				work_space_circles.push_back(Circle(center_global, rs.radius));
			}
		}

		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}
}