#include "window.hpp"

#include <iostream>
#include <cmath>

#include <chrono>

std::vector<Window::SDLPointers> Window::allSdlPointers = std::vector<SDLPointers>(0);
bool Window::sdlIsInitialized = false;

Window::Window(const char* name, float screen_size_in_distance_units,
	int screen_size_in_pixels, float frame_rate_in_Hz, bool* is_good)
	:
	screenSizeInDistanceUnits(screen_size_in_distance_units),
	screenSizeInPixels(screen_size_in_pixels),
	pixelsPerDistanceUnit(screen_size_in_pixels/screen_size_in_distance_units),
	sdlWindowCreationNumber(allSdlPointers.size()),
	frameRateInHz(frame_rate_in_Hz),
	half_plane_renderer(*this),
	halfplanes_areas_time(0.f),
	halfplanes_borders_time(0.f),
	points_time(0.f),
	circles_time(0.f),
	arrows_time(0.f),
	n_frames(0),
	render_feasible_region(false)
{
	SDLPointers sdl_pointers;
	sdl_pointers.window = 0;
	sdl_pointers.renderer = 0;
	allSdlPointers.push_back(sdl_pointers);

	*is_good = false;
	if (!sdlIsInitialized && (SDL_Init(SDL_INIT_VIDEO) < 0))
	{
		std::cout << "Could not initialize SDL." << std::endl;
		return;
	}
	sdlIsInitialized = true;

	allSdlPointers[sdlWindowCreationNumber].window = SDL_CreateWindow(name, SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED, screenSizeInPixels, screenSizeInPixels, SDL_WINDOW_SHOWN);
	if (allSdlPointers[sdlWindowCreationNumber].window == 0)
	{
		std::cout << "Could not create window. SDL_Error: " << SDL_GetError() << std::endl;
		return;
	}

	allSdlPointers[sdlWindowCreationNumber].renderer = SDL_CreateRenderer(
		allSdlPointers[sdlWindowCreationNumber].window, -1, SDL_RENDERER_ACCELERATED);
	if (allSdlPointers[sdlWindowCreationNumber].renderer == 0)
	{
		std::cout << "Could not create renderer. SDL_Error: " << SDL_GetError() << std::endl;
		return;
	}
	*is_good = true;
}

Window::~Window()
{
	killSdlWindow(sdlWindowCreationNumber);

	// check if this is the last object which had a living window and quit sdl in that case
	bool all_dead = true;
	for (int i = 0; i < allSdlPointers.size(); i++)
	{
		if (allSdlPointers[i].window)
			all_dead = false;
	}
	if (all_dead)
	{
		sdlIsInitialized = false;
		SDL_Quit();
	}

	std::cout << "N_frames " << n_frames << std::endl;
	std::cout << "halfplanes_areas_time " << halfplanes_areas_time << std::endl;
	std::cout << "halfplanes_borders_time " << halfplanes_borders_time << std::endl;
	std::cout << "points_time " << points_time << std::endl;
	std::cout << "circles_time " << circles_time << std::endl;
	std::cout << "arrows_time " << arrows_time << std::endl;
}

void Window::killSdlWindow(int window_creation_number)
{
	if (allSdlPointers[window_creation_number].renderer)
	{
		SDL_DestroyRenderer(allSdlPointers[window_creation_number].renderer);
		allSdlPointers[window_creation_number].renderer = 0;
	}

	if (allSdlPointers[window_creation_number].window)
	{
		SDL_DestroyWindow(allSdlPointers[window_creation_number].window);
		allSdlPointers[window_creation_number].window = 0;
	}
}

bool Window::sdlCheck()
{
	// process events, kill windows if they have been closed
	SDL_Event e;
	while(SDL_PollEvent(&e) != 0)
	{
		if (e.type == SDL_QUIT) // kill all windows
		{
			//std::cout << "SDL-quit event" << std::endl;
			for (int i = 0; i < allSdlPointers.size(); i++)
				killSdlWindow(i);
		}
		else if (e.type == SDL_WINDOWEVENT &&
			e.window.event == SDL_WINDOWEVENT_CLOSE) // kill the respective window
		{
			//std::cout << "Window-close event" << std::endl;
			for (int i = 0; i < allSdlPointers.size(); i++)
			{
				if (SDL_GetWindowID(allSdlPointers[i].window) == e.window.windowID)
					killSdlWindow(i);
			}	
		}
	}
	// check if some object has killed this object's sdl window and return true/false
	if (allSdlPointers[sdlWindowCreationNumber].window == 0)
	{
		//std::cout << "Window has been killed" << std::endl;
		return true;
	}
	else
		return false;
}

float Window::delayUntilNextFrameInMinutes()
{
	SDL_Delay(1000.0/frameRateInHz);
	return 1.0/frameRateInHz/60.0;
}

bool Window::render(const std::vector<Geometry2D::HalfPlane2>* half_planes_ptr,
		const std::vector<Geometry2D::Vec2>* points_ptr,
		const std::vector<sdlColor>& points_colors,
		const std::vector<AdditionalPrimitives2D::Circle>* circles_ptr,
		const std::vector<AdditionalPrimitives2D::Arrow>* arrows_ptr,
		const std::vector<Window::sdlColor>& arrows_colors,
		const std::vector<Window::sdlColor>& circles_colors,
		const std::vector<AdditionalPrimitives2D::Polygon>* polygons_ptr,
		const std::vector<Geometry2D::Capsule>* capsules_ptr)
{
	n_frames++;
	if (sdlCheck())
		return true;
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;
	// make the screen black
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	// for half planes
	if (half_planes_ptr)
		renderHalfPlanes(*half_planes_ptr);
	// for points
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	float radius = 0.005*screenSizeInDistanceUnits;
	if (points_ptr)
	{
		const std::vector<Geometry2D::Vec2>& points = *points_ptr;
		for (int i = 0; i < points.size(); i++)
		{
			if (i < points_colors.size())
				renderCircle(points[i], radius, 0.0, points_colors[i]);
			else
				renderCircle(points[i], radius, 0.0);
		}
	}
	t2 = std::chrono::high_resolution_clock::now();
	points_time = points_time*(n_frames-1)/n_frames +
		(std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1)).count()/n_frames;
	// for circles
	t1 = std::chrono::high_resolution_clock::now();
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	float thickness = 0.0025*screenSizeInDistanceUnits;
	if (circles_ptr)
	{
		const std::vector<AdditionalPrimitives2D::Circle>& circles = *circles_ptr;
		for (int i = 0; i < circles.size(); i++)
		{
			if (circles_colors.size() > i)
				renderCircle(circles[i].center, circles[i].radius, circles[i].radius - thickness, circles_colors[i]);
			else
				renderCircle(circles[i].center, circles[i].radius, circles[i].radius - thickness);
		}
	}
	t2 = std::chrono::high_resolution_clock::now();
	circles_time = circles_time*(n_frames-1)/n_frames +
		(std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1)).count()/n_frames;
	// for arrows
	t1 = std::chrono::high_resolution_clock::now();
	if (arrows_ptr)
		renderArrows(*arrows_ptr, arrows_colors);
	t2 = std::chrono::high_resolution_clock::now();
	arrows_time = arrows_time*(n_frames-1)/n_frames +
		(std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1)).count()/n_frames;
	
	if (polygons_ptr)
		renderPolygons(*polygons_ptr);

	if (capsules_ptr)
		renderCapsules(*capsules_ptr);

	// bring it to the screen
	SDL_RenderPresent(renderer);
	return false;
}

Window::Pixel::Pixel() : i(0), j(0) {}
		
Window::Pixel::Pixel(int i, int j) : i(i), j(j) {}

Window::Pixel Window::pointToPixel(const Geometry2D::Vec2& point) const
{
	Pixel p;
	p.i = point.x*pixelsPerDistanceUnit + screenSizeInPixels/2;
	p.j = -point.y*pixelsPerDistanceUnit + screenSizeInPixels/2;
	return p;
}

Window::Pixel Window::unitVectorToPixelDirection(const Geometry2D::Vec2& unit_vector)
{
	Pixel p;
	if (unit_vector.x > 0.0)
		p.i = 1;
	else
		p.i = -1;
	if (unit_vector.y > 0.0)
		p.j = -1;
	else
		p.j = 1;
	return p;
}

Window::BoundingBox Window::circleToBoundingBox(const Geometry2D::Vec2& center, float radius)
{
	BoundingBox bb;
	bb.lower_bounds = pointToPixel(Geometry2D::Vec2(center.x-radius, center.y+radius));
	bb.upper_bounds = pointToPixel(Geometry2D::Vec2(center.x+radius, center.y-radius));
	return bb;
}

Geometry2D::Vec2 Window::pixelToPoint(const Pixel& p) const
{
	return Geometry2D::Vec2((p.i - screenSizeInPixels/2 + 0.5)/pixelsPerDistanceUnit,
		-(p.j - screenSizeInPixels/2 + 0.5)/pixelsPerDistanceUnit);
}

void Window::renderCircle(const Geometry2D::Vec2& center, float r_outer, float r_inner, const Window::sdlColor& color)
{
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;

	SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);

	BoundingBox bb = circleToBoundingBox(center, r_outer);
	Geometry2D::Vec2 point;
	float distance;
	for (int i = bb.lower_bounds.i; i <= bb.upper_bounds.i; i++)
	{
		for (int j = bb.lower_bounds.j; j <= bb.upper_bounds.j; j++)
		{
			point = pixelToPoint(Pixel(i, j));
			distance = std::sqrt((point - center).dot(point - center));
			if (distance <= r_outer && distance >= r_inner)
				SDL_RenderDrawPoint(renderer, i, j);
		}
	}
}

void Window::renderCutCircle(const Geometry2D::Vec2& center, float r_outer, float r_inner,
	const Geometry2D::HalfPlane2& h_cut, const Window::sdlColor& color)
{
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;

	SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);

	BoundingBox bb = circleToBoundingBox(center, r_outer);
	Geometry2D::Vec2 point;
	float distance;
	for (int i = bb.lower_bounds.i; i <= bb.upper_bounds.i; i++)
	{
		for (int j = bb.lower_bounds.j; j <= bb.upper_bounds.j; j++)
		{
			point = pixelToPoint(Pixel(i, j));
			distance = std::sqrt((point - center).dot(point - center));
			if ((distance <= r_outer && distance >= r_inner) && h_cut.signedDistance(point) > 0.f)
				SDL_RenderDrawPoint(renderer, i, j);
		}
	}
}

void Window::renderHalfPlanes(const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;
	
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	// render the complementary half-planes, in grey
	//SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
	/*Geometry2D::Vec2 point;
	for (int i = 0; i < screenSizeInPixels; i++)
	{
		for (int j = 0; j < screenSizeInPixels; j++)
		{
			point = pixelToPoint(Pixel(i, j));
			for (int k = 0; k < half_planes.size(); k++)
			{
				if (half_planes[k].signedDistance(point) > 0.0)
				{
					SDL_RenderDrawPoint(renderer, i, j);
					break;
				}
			}
		}	
	}*/
	//half_plane_renderer.render(renderer, half_planes);
	if (render_feasible_region)
		half_plane_renderer.divideAndConquerRendering(renderer, half_planes);
	t2 = std::chrono::high_resolution_clock::now();
	halfplanes_areas_time = halfplanes_areas_time*(n_frames-1)/n_frames +
		(std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1)).count()/n_frames;
	// render boundary lines shifted outwards (into the "infeasible" space), in red
	t1 = std::chrono::high_resolution_clock::now();
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
	for (int i = 0; i < half_planes.size(); i++)
		renderBoundaryLine(half_planes[i], 2);
	// render boundary lines, in white
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	for (int i = 0; i < half_planes.size(); i++)
		renderBoundaryLine(half_planes[i]);
	t2 = std::chrono::high_resolution_clock::now();
	halfplanes_borders_time = halfplanes_borders_time*(n_frames-1)/n_frames +
		(std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1)).count()/n_frames;
}

void Window::renderBoundaryLine(const Geometry2D::HalfPlane2& half_plane, int pixel_shift)
{
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer; //for convenience
	Geometry2D::Vec2 e_line(half_plane.getParallel());
	Geometry2D::Vec2 anchor = half_plane.getNormal()*half_plane.getOffset();
	Pixel p1(pointToPixel(anchor + e_line*2*screenSizeInDistanceUnits));
	Pixel p2(pointToPixel(anchor - e_line*2*screenSizeInDistanceUnits));
	Pixel shift_vector = unitVectorToPixelDirection(half_plane.getNormal());
	SDL_RenderDrawLine(renderer, p1.i + shift_vector.i*pixel_shift, p1.j + shift_vector.j*pixel_shift,
		p2.i + shift_vector.i*pixel_shift, p2.j + shift_vector.j*pixel_shift);
}

void Window::renderArrows(const std::vector<AdditionalPrimitives2D::Arrow>& arrows,
	const std::vector<Window::sdlColor>& arrows_colors)
{
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer; // for convenience
	for (int i = 0; i < arrows.size(); i++)
	{
		if (i < arrows_colors.size())
		{
			SDL_SetRenderDrawColor(renderer, arrows_colors[i].r, arrows_colors[i].g,
					arrows_colors[i].b, 255);
		}
		else
			SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		Geometry2D::Vec2 d = arrows[i].head - arrows[i].tail;
		if (d.norm() > 0.00001f)
		{
			Geometry2D::Vec2 normal(Geometry2D::Vec2(-d.y, d.x).normalized());
			Pixel shift_vector = unitVectorToPixelDirection(normal);
			Pixel head(pointToPixel(arrows[i].head));
			Pixel tail(pointToPixel(arrows[i].tail));
			SDL_RenderDrawLine(renderer, tail.i, tail.j, head.i, head.j);
			for (int k = 0; k < int(0.006*screenSizeInPixels); k++)
			{
				SDL_RenderDrawLine(renderer, tail.i, tail.j, head.i+(k+1)*shift_vector.i, head.j+(k+1)*shift_vector.j);
				SDL_RenderDrawLine(renderer, tail.i, tail.j, head.i-(k+1)*shift_vector.i, head.j-(k+1)*shift_vector.j);
			}
		}
		renderCircle(arrows[i].head, 0.009*screenSizeInDistanceUnits, 0.0);
	}
}

void Window::renderPolygons(const std::vector<AdditionalPrimitives2D::Polygon>& polygons)
{
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer; // for convenience
	for (auto& pg : polygons)
	{
		for (int i = 1; i < pg.size(); i++)
		{
			SDL_RenderDrawLine(renderer, pointToPixel(pg[i]).i, pointToPixel(pg[i]).j,
				pointToPixel(pg[i - 1]).i, pointToPixel(pg[i - 1]).j);
		}
		if (pg.size() > 1)
		{
			SDL_RenderDrawLine(renderer, pointToPixel(pg[0]).i, pointToPixel(pg[0]).j,
				pointToPixel(pg.back()).i, pointToPixel(pg.back()).j);
		}
	}
}

void Window::renderCapsules(const std::vector<Geometry2D::Capsule>& capsules)
{
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer; // for convenience
	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // render capsules in green
	for (auto& c : capsules)
	{
		Geometry2D::Vec2 centers_diff = c.center_a() - c.center_b();
		Geometry2D::Vec2 normal(-centers_diff.normalized().y, centers_diff.normalized().x);
		Geometry2D::Vec2 a1 = c.center_a() + c.radius()*normal;
		Geometry2D::Vec2 a2 = c.center_a() - c.radius()*normal;
		Geometry2D::Vec2 b1 = c.center_b() + c.radius()*normal;
		Geometry2D::Vec2 b2 = c.center_b() - c.radius()*normal;
		SDL_RenderDrawLine(renderer, pointToPixel(a1).i, pointToPixel(a1).j,
			pointToPixel(b1).i, pointToPixel(b1).j);
		SDL_RenderDrawLine(renderer, pointToPixel(a2).i, pointToPixel(a2).j,
			pointToPixel(b2).i, pointToPixel(b2).j);
		
		float thickness = 0.0025*screenSizeInDistanceUnits;
		Window::sdlColor green;
		green.r = green.b = 0;
		green.g = 255;
		renderCutCircle(c.center_a(), c.radius(), c.radius() - thickness, c.bound_a(), green);
		renderCutCircle(c.center_b(), c.radius(), c.radius() - thickness, c.bound_b(), green);
	}
}

/*
Window::HalfPlaneRenderer::HalfPlaneRenderer(const Window& win,
	const std::vector<Geometry2D::HalfPlane2>& halfplanes)
	: lower_left_corner_bounding_box(-win.screenSizeInDistanceUnits-1.f, -win.screenSizeInDistanceUnits-1.f)
	, upper_left_corner_bounding_box(-win.screenSizeInDistanceUnits-1.f, win.screenSizeInDistanceUnits+1.f)
	, lower_right_corner_bounding_box(win.screenSizeInDistanceUnits+1.f, -win.screenSizeInDistanceUnits-1.f)
	, upper_right_corner_bounding_box(win.screenSizeInDistanceUnits+1.f, win.screenSizeInDistanceUnits+1.f)
	, binary_points(win.screenSizeInPixels*win.screenSizeInPixels)
	, infeasible_points(new SDL_Point[win.screenSizeInPixels*win.screenSizeInPixels])
	, count_infeasible(0)
{
	for (int i = 0; i < win.screenSizeInPixels; i++)
	{
		for (int j = 0; j < win.screenSizeInPixels; j++)
		{
			binary_points[i*win.screenSizeInPixels + j].point = win.pixelToPoint(Pixel(i,j));
			binary_points[i*win.screenSizeInPixels + j].feasible = true;
		}
	}

	for (auto& h : halfplanes)
	{
		if ((h.signedDistance(lower_left_corner_bounding_box) < 0.f) &&
			(h.signedDistance(upper_left_corner_bounding_box) < 0.f) &&
			(h.signedDistance(lower_right_corner_bounding_box) < 0.f) &&
			(h.signedDistance(upper_right_corner_bounding_box) < 0.f))
			continue;

		lower_left_corner_bounding_box = Geometry2D::Vec2(win.screenSizeInDistanceUnits+1.f, win.screenSizeInDistanceUnits+1.f);
		upper_left_corner_bounding_box = Geometry2D::Vec2(win.screenSizeInDistanceUnits+1.f, -win.screenSizeInDistanceUnits-1.f);
		lower_right_corner_bounding_box = Geometry2D::Vec2(-win.screenSizeInDistanceUnits-1.f, win.screenSizeInDistanceUnits+1.f);
		upper_right_corner_bounding_box = Geometry2D::Vec2(-win.screenSizeInDistanceUnits-1.f, -win.screenSizeInDistanceUnits-1.f);

		for (std::vector<BinaryPoint>::iterator it = binary_points.begin(); it != binary_points.end(); it++)
		{
			if (it->feasible && (h.signedDistance(it->point) > 0.f))
			{
				infeasible_points[count_infeasible].x = win.pointToPixel(it->point).i;
				infeasible_points[count_infeasible].y = win.pointToPixel(it->point).j;
				count_infeasible++;
				it->feasible = false;
			}
			else
			{
				if (it->point.x < lower_left_corner_bounding_box.x)
				{
					lower_left_corner_bounding_box.x = it->point.x;
					upper_left_corner_bounding_box.x = it->point.x;
				}
				if (it->point.x > lower_right_corner_bounding_box.x)
				{
					lower_right_corner_bounding_box.x = it->point.x;
					upper_right_corner_bounding_box.x = it->point.x;
				}
				if (it->point.y < lower_left_corner_bounding_box.y)
				{
					lower_left_corner_bounding_box.y = it->point.y;
					lower_right_corner_bounding_box.y = it->point.y;
				}
				if (it->point.y > upper_left_corner_bounding_box.y)
				{
					upper_left_corner_bounding_box.y = it->point.y;
					upper_right_corner_bounding_box.y = it->point.y;
				}
			}
		}
	}
}
*/

Window::HalfPlaneRenderer::HalfPlaneRenderer(const Window& win)
	: win(win)
	, infeasible_points(new SDL_Point[win.screenSizeInPixels*win.screenSizeInPixels*2])
	, close_to_infeasible_points(new SDL_Point[win.screenSizeInPixels*win.screenSizeInPixels*2])
	, count_infeasible(0)
	, count_close_to_infeasible(0)
	, closeness_threshold(-0.002*win.screenSizeInDistanceUnits)
{ }

Window::HalfPlaneRenderer::~HalfPlaneRenderer()
{
	delete[] infeasible_points;
}

void Window::HalfPlaneRenderer::render(SDL_Renderer* renderer, const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	count_infeasible = 0;
	for (int i = 0; i < win.screenSizeInPixels; i++)
	{
		for (int j = 0; j < win.screenSizeInPixels; j++)
		{
			for (int k = 0; k < half_planes.size(); k++)
			{
				if (half_planes[k].signedDistance(win.pixelToPoint(Pixel(i, j))) > 0.0)
				{
					infeasible_points[count_infeasible].x = i;
					infeasible_points[count_infeasible].y = j;
					count_infeasible++;
					break;
				}
			}
		}	
	}
	SDL_RenderDrawPoints(renderer, infeasible_points, count_infeasible);
}

void Window::HalfPlaneRenderer::divideAndConquerRendering(SDL_Renderer* renderer,
	const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	count_infeasible = 0;
	count_close_to_infeasible = 0;
	subdivideAndRender(renderer, half_planes, 8,
		-win.screenSizeInDistanceUnits/2.f, win.screenSizeInDistanceUnits/2.f,
		-win.screenSizeInDistanceUnits/2.f, win.screenSizeInDistanceUnits/2.f);

	SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
	SDL_RenderDrawPoints(renderer, infeasible_points, count_infeasible);
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderDrawPoints(renderer, close_to_infeasible_points, count_close_to_infeasible);
}

void Window::HalfPlaneRenderer::subdivideAndRender(SDL_Renderer* renderer,
	const std::vector<Geometry2D::HalfPlane2>& half_planes,
	int n_sub_divide, float lower_x, float upper_x, float lower_y, float upper_y)
{
	int result = boxFeasibility(lower_x, upper_x, lower_y, upper_y, half_planes);
	if (result == 1)
		renderBox(renderer, true, lower_x, upper_x, lower_y, upper_y);
	else if (result == -1)
		renderBox(renderer, false, lower_x, upper_x, lower_y, upper_y);
	else if (n_sub_divide > 0)
	{
		subdivideAndRender(renderer, half_planes, n_sub_divide -1,
			lower_x, (lower_x + upper_x)/2.f, lower_y, (lower_y + upper_y)/2.f);
		subdivideAndRender(renderer, half_planes, n_sub_divide -1,
			(lower_x + upper_x)/2.f, upper_x, lower_y, (lower_y + upper_y)/2.f);
		subdivideAndRender(renderer, half_planes, n_sub_divide -1,
			lower_x, (lower_x + upper_x)/2.f, (lower_y + upper_y)/2.f, upper_y);
		subdivideAndRender(renderer, half_planes, n_sub_divide -1,
			(lower_x + upper_x)/2.f, upper_x, (lower_y + upper_y)/2.f, upper_y);
	}
	else
	{
		SDL_Rect rect;
		createPixelRectangle(lower_x, upper_x, lower_y, upper_y, &rect);
		for (int i = rect.x; i <= rect.x + rect.w; i++)
		{
			for (int j = rect.y; j <= rect.y + rect.h; j++)
			{
				int point_feasibility = pointFeasibility(win.pixelToPoint(Pixel(i, j)), half_planes);
				if (point_feasibility == -1)
				{
					infeasible_points[count_infeasible].x = i;
					infeasible_points[count_infeasible].y = j;
					count_infeasible++;
				}
				else if (point_feasibility == 0)
				{
					close_to_infeasible_points[count_close_to_infeasible].x = i;
					close_to_infeasible_points[count_close_to_infeasible].y = j;
					count_close_to_infeasible++;
				}
			}
		}
	}
}

int Window::HalfPlaneRenderer::pointFeasibility(const Geometry2D::Vec2& point,
	const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	float closeness = -10.f*win.screenSizeInDistanceUnits;
	for (int k = 0; k < half_planes.size(); k++)
	{
		if (half_planes[k].signedDistance(point) > 0.0)
			return -1;
		else if (half_planes[k].signedDistance(point) > closeness)
			closeness = half_planes[k].signedDistance(point);
	}
	if (closeness > closeness_threshold)
		return 0;
	return 1;
}

int Window::HalfPlaneRenderer::boxFeasibility(float lower_x, float upper_x, float lower_y, float upper_y,
	const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	Geometry2D::Vec2 p_lower_x_lower_y(lower_x, lower_y);
	Geometry2D::Vec2 p_upper_x_lower_y(upper_x, lower_y);
	Geometry2D::Vec2 p_lower_x_upper_y(lower_x, upper_y);
	Geometry2D::Vec2 p_upper_x_upper_y(upper_x, upper_y);
	bool found_infeasible_or_close_to_infeasible_point = false;
	for (int k = 0; k < half_planes.size(); k++)
	{
		if ((half_planes[k].signedDistance(p_lower_x_lower_y) > closeness_threshold) ||
			(half_planes[k].signedDistance(p_upper_x_lower_y) > closeness_threshold) ||
			(half_planes[k].signedDistance(p_lower_x_upper_y) > closeness_threshold) ||
			(half_planes[k].signedDistance(p_upper_x_upper_y) > closeness_threshold))
			found_infeasible_or_close_to_infeasible_point = true;
		if ((half_planes[k].signedDistance(p_lower_x_lower_y) > 0.0) &&
			(half_planes[k].signedDistance(p_upper_x_lower_y) > 0.0) &&
			(half_planes[k].signedDistance(p_lower_x_upper_y) > 0.0) &&
			(half_planes[k].signedDistance(p_upper_x_upper_y) > 0.0))
			return -1;
	}
	if (found_infeasible_or_close_to_infeasible_point)
		return 0;
	else
		return 1;
}

void Window::HalfPlaneRenderer::renderBox(SDL_Renderer* renderer, bool feasible,
	float lower_x, float upper_x, float lower_y, float upper_y)
{
	if (feasible)
		return;
	else
	{
		SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
		SDL_Rect rect;
		createPixelRectangle(lower_x, upper_x, lower_y, upper_y, &rect);
		SDL_RenderFillRect(renderer, &rect);
	}
}

void Window::HalfPlaneRenderer::createPixelRectangle(float lower_x, float upper_x, float lower_y, float upper_y,
	SDL_Rect* rectangle)
{
	SDL_Rect& rect = *rectangle;
	rect.x = win.pointToPixel(Geometry2D::Vec2(lower_x, upper_y)).i;
	rect.y = win.pointToPixel(Geometry2D::Vec2(lower_x, upper_y)).j;
	rect.w = win.pointToPixel(Geometry2D::Vec2(upper_x, upper_y)).i - rect.x;
	rect.h = win.pointToPixel(Geometry2D::Vec2(lower_x, lower_y)).j - rect.y;
	return;
	if (win.pixelToPoint(Pixel(rect.x - 1, rect.y)).x >= lower_x)
	{
		rect.x--;
		rect.w++;
	}
	if (win.pixelToPoint(Pixel(rect.x, rect.y)).x < lower_x)
	{
		rect.x++;
		rect.w--;
	}
	if (win.pixelToPoint(Pixel(rect.x, rect.y + 1)).y <= upper_y)
	{
		rect.y--;
		rect.h++;
	}
	if (win.pixelToPoint(Pixel(rect.x, rect.y)).y > upper_y)
	{
		rect.y++;
		rect.h--;
	}
	if (win.pixelToPoint(Pixel(rect.x + rect.w + 1, rect.y + rect.h)).x <= upper_x)
		rect.w++;
	if (win.pixelToPoint(Pixel(rect.x + rect.w, rect.y + rect.h)).x > upper_x)
		rect.w--;
	if (win.pixelToPoint(Pixel(rect.x + rect.w, rect.y + rect.h + 1)).y >= lower_y)
		rect.h++;
	if (win.pixelToPoint(Pixel(rect.x + rect.w, rect.y + rect.h)).y < lower_y)
		rect.h--;
}