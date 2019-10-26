#include "gui.hpp"

GUI::GUI(float minutes_to_run,
	const std::vector<Geometry2D::HalfPlane2>& half_planes,
	const std::vector<Geometry2D::Vec2>& points,
	const std::vector<Geometry2D::Circle2>& circles,
	const std::vector<Geometry2D::AxisAlignedBoundingBox2>& aabbs)
{
	bool window_is_good;
	window = new Window("GUI", 4.0, 1000, 10.0, &window_is_good);
	if (!window_is_good)
	{
		delete window;
		window = 0;
		return;
	}

	// give a green color to all the points
	std::vector<Window::sdlColor> points_colors;
	Window::sdlColor green;
	green.r = green.b = 0;
	green.g = 255;
	for (int i = 0; i < points.size(); i++)
		points_colors.push_back(green);

	if (minutes_to_run > 0.0)
	{
		// display the points and half-planes for minutes_to_run [minutes]
		float remaining_minutes = minutes_to_run;
		while (remaining_minutes > 0.0 &&
			!window->render(half_planes, points, points_colors, circles, aabbs))
		{
			remaining_minutes -= window->delayUntilNextFrameInMinutes();
		}
	}
	else
	{
		// display the points and half-planes until the window is closed
		while (!window->render(half_planes, points, points_colors, circles, aabbs))
		{
			window->delayUntilNextFrameInMinutes();
		}
	}

	// delete the window
	delete window;
	window = 0;
}

GUI::~GUI()
{
	if (window)
	{
		delete window;
		window = 0;
	}
}