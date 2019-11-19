#include "../src/geometry.hpp"

#include <gtest/gtest.h>
#include <iostream>

/*
TEST(AABBLineIntersection, CornerCases)
{
	Geometry2D::AxisAlignedBoundingBox2 aabb1(0.0, 1.0, 0.0, 1.0);
	Geometry2D::HalfPlane2 h1(Geometry2D::Vec2(1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.0);
	Geometry2D::AABBLineIntersection result1 = aabb1.intersectWithHalfPlaneBoundary(h1);
	EXPECT_EQ(result1, Geometry2D::AABBLineIntersection::x_lower_y_lower) <<
		"A single intersection in the lower left corner is not in the category x_lower_y_lower" << std::endl;

	Geometry2D::AxisAlignedBoundingBox2 aabb2(-1.0, 0.0, -1.0, 0.0);
	Geometry2D::HalfPlane2 h2(Geometry2D::Vec2(1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.0);
	Geometry2D::AABBLineIntersection result2 = aabb2.intersectWithHalfPlaneBoundary(h2);
	EXPECT_EQ(result2, Geometry2D::AABBLineIntersection::x_upper_y_upper) <<
		"A single intersection in the upper right corner is not in the category x_upper_y_upper" << std::endl;

	Geometry2D::AxisAlignedBoundingBox2 aabb3(0.0, 1.0, -1.0, 0.0);
	Geometry2D::HalfPlane2 h3(Geometry2D::Vec2(-1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.0);
	Geometry2D::AABBLineIntersection result3 = aabb3.intersectWithHalfPlaneBoundary(h3);
	EXPECT_EQ(result3, Geometry2D::AABBLineIntersection::x_lower_y_upper) <<
		"A single intersection in the upper left corner is not in the category x_lower_y_upper" << std::endl;

	Geometry2D::AxisAlignedBoundingBox2 aabb4(-1.0, 0.0, 0.0, 1.0);
	Geometry2D::HalfPlane2 h4(Geometry2D::Vec2(-1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.0);
	Geometry2D::AABBLineIntersection result4 = aabb4.intersectWithHalfPlaneBoundary(h4);
	EXPECT_EQ(result4, Geometry2D::AABBLineIntersection::x_upper_y_lower) <<
		"A single intersection in the lower right corner is not in the category x_upper_y_lower" << std::endl;
}

TEST(AABBLineIntersection, EdgeCases)
{
}
*/

TEST(AABBLineIntersection, StandardCases)
{
	Geometry2D::AxisAlignedBoundingBox2 aabb(-1.0, 1.0, -1.0, 1.0);
	Geometry2D::HalfPlane2 h1(Geometry2D::Vec2(1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.1);
	Geometry2D::HalfPlane2 h2(Geometry2D::Vec2(-1.0,1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.1);
	Geometry2D::HalfPlane2 h3(Geometry2D::Vec2(1.0,-1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.1);
	Geometry2D::HalfPlane2 h4(Geometry2D::Vec2(-1.0,-1.0)/Geometry2D::Vec2(1.0,1.0).norm(), 0.1);
	Geometry2D::HalfPlane2 h5(Geometry2D::Vec2(1.0,0.0), 0.0);
	Geometry2D::HalfPlane2 h6(Geometry2D::Vec2(0.0,1.0), 0.0);

	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h1), Geometry2D::AABBLineIntersection::x_upper_y_upper);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h1.flip()), Geometry2D::AABBLineIntersection::x_upper_y_upper);
	
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h2), Geometry2D::AABBLineIntersection::x_lower_y_upper);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h2.flip()), Geometry2D::AABBLineIntersection::x_lower_y_upper);
	
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h3), Geometry2D::AABBLineIntersection::x_upper_y_lower);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h3.flip()), Geometry2D::AABBLineIntersection::x_upper_y_lower);
	
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h4), Geometry2D::AABBLineIntersection::x_lower_y_lower);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h4.flip()), Geometry2D::AABBLineIntersection::x_lower_y_lower);

	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h5), Geometry2D::AABBLineIntersection::y_lower_y_upper);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h5.flip()), Geometry2D::AABBLineIntersection::y_lower_y_upper);

	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h6), Geometry2D::AABBLineIntersection::x_lower_x_upper);
	EXPECT_EQ(aabb.intersectWithHalfPlaneBoundary(h6.flip()), Geometry2D::AABBLineIntersection::x_lower_x_upper);
}

//TEST()
//{}