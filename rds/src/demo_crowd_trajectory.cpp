#include "crowd_trajectory.hpp"
#include "geometry.hpp"
#include "gui.hpp"
#include "spline.h" // from https://kluge.in-chemnitz.de/opensource/spline/
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

using Geometry2D::Vec2;

int main(int argc, char** argv)
{
	//if (argc < 2)
	//	std::cout << "No argument. Require 1 argument (the data file name)." << std::endl;
	char file_name[] = "./data_university_students/students003_no_obstacles.vsp";
	float frame_rate = 20.f;
	CrowdTrajectory c(file_name, frame_rate);
	std::cout << "Number of splines = " << c.getSplinesData().size() << std::endl;

	if (false)
	{
		std::vector<Vec2> points;
		std::vector<GuiColor> points_colors;
		GuiColor white, red;
		white.r = white.g = white.b = 255;
		red.r = 255;
		red.g = red.b = 0;
		float max_abs_coordinate = 0.f;
		int count = 0;
		for (auto& d : c.getSplinesData())
		{
			float t_min = 10000000.0;
			float t_max = -t_min;
			std::vector<double> T, X, Y;
			for (auto& knot : d) // [3] shows a weird loop
			{
				T.push_back(knot.t);
				X.push_back(knot.p.x);
				Y.push_back(knot.p.y);
				if (max_abs_coordinate < std::abs(knot.p.x))
					max_abs_coordinate = std::abs(knot.p.x);
				if (max_abs_coordinate < std::abs(knot.p.y))
					max_abs_coordinate = std::abs(knot.p.y);
				if (t_min > knot.t)
					t_min = knot.t;
				if (t_max < knot.t)
					t_max = knot.t;
			}
			tk::spline sx, sy;
			sx.set_points(T, X);
			sy.set_points(T, Y);
			for (int i = -10; i < 110; i++)
			{
				float t = t_min + i/100.f*(t_max - t_min);
				points.push_back(Vec2(sx(t), sy(t)));
				points_colors.push_back(red);
			}
			for (int i = 0; i < X.size(); i++)
			{
				points.push_back(Vec2(X[i], Y[i]));
				points_colors.push_back(white);
			}
			count++;
			if (count == 50)
				break;
		}

		GUI gui("Spline Test", max_abs_coordinate*2.1f);
		gui.points = &points;
		gui.points_colors = points_colors;
		gui.blockingShowUntilClosed();
	}

	std::vector<Vec2> points_pedestrians(2*c.getNumSplines());
	GUI gui("Crowd Trajectory Test", 2000.f);
	gui.points = &points_pedestrians;
	GuiColor red, white;
	red.g = red.b = 0;
	for (unsigned int i = 0; i < c.getNumSplines(); i++)
	{
		gui.points_colors.push_back(white);
		gui.points_colors.push_back(red);
	}

	float dt = 1.f/frame_rate;
	std::chrono::milliseconds gui_cycle_time(int(dt*1000.f));
	std::chrono::high_resolution_clock::time_point t_gui_update = std::chrono::high_resolution_clock::now();
	float t = 0;
	do
	{
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - t_gui_update));
		t_gui_update = std::chrono::high_resolution_clock::now();
		for (unsigned int i = 0; i < c.getNumSplines(); i++)
		{
			c.getPedestrianPositionAtTime(i, t, &points_pedestrians[i*2]);
			Vec2 velocity;
			c.getPedestrianVelocityAtTime(i, t, &velocity);
			points_pedestrians[i*2 + 1] = points_pedestrians[i*2] + 0.2f*velocity;
		}
		t += dt;
	}
	while (gui.update() == 0);

	return 0;
}