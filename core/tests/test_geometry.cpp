#include <gtest/gtest.h>

#include <cmath>

#include "geofence_guard/geometry.hpp"

using gfg::Polygon;
using gfg::Vec2;

namespace {

const Polygon kSquare = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
// An L: the square [0,20]^2 with the quadrant x>10, y>10 removed.
const Polygon kEll = {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}};

}  // namespace

TEST(PointInPolygon, InsideAndOutside) {
  EXPECT_TRUE(gfg::point_in_polygon({5, 5}, kSquare));
  EXPECT_FALSE(gfg::point_in_polygon({15, 5}, kSquare));
  EXPECT_FALSE(gfg::point_in_polygon({-0.001, 5}, kSquare));
}

TEST(PointInPolygon, NonConvexNotch) {
  EXPECT_TRUE(gfg::point_in_polygon({5, 15}, kEll));
  EXPECT_TRUE(gfg::point_in_polygon({15, 5}, kEll));
  EXPECT_FALSE(gfg::point_in_polygon({15, 15}, kEll));
}

TEST(PointInPolygon, OrientationDoesNotMatter) {
  const Polygon cw(kEll.rbegin(), kEll.rend());
  EXPECT_TRUE(gfg::point_in_polygon({5, 15}, cw));
  EXPECT_FALSE(gfg::point_in_polygon({15, 15}, cw));
}

TEST(PointInPolygon, RayThroughVertexCountsOnce) {
  // y = 10 passes exactly through two vertices of the L.
  EXPECT_TRUE(gfg::point_in_polygon({5, 10}, kEll));
  EXPECT_FALSE(gfg::point_in_polygon({-5, 10}, kEll));
}

TEST(DistancePointSegment, ProjectsOntoInterior) {
  EXPECT_DOUBLE_EQ(gfg::distance_point_segment({5, 3}, {0, 0}, {10, 0}), 3.0);
}

TEST(DistancePointSegment, ClampsToEndpoints) {
  EXPECT_DOUBLE_EQ(gfg::distance_point_segment({13, 4}, {0, 0}, {10, 0}), 5.0);
  EXPECT_DOUBLE_EQ(gfg::distance_point_segment({-3, -4}, {0, 0}, {10, 0}), 5.0);
}

TEST(DistancePointSegment, DegenerateSegmentIsAPoint) {
  EXPECT_DOUBLE_EQ(gfg::distance_point_segment({3, 4}, {0, 0}, {0, 0}), 5.0);
}

TEST(SegmentsIntersect, Crossing) {
  EXPECT_TRUE(gfg::segments_intersect({0, 0}, {10, 10}, {0, 10}, {10, 0}));
}

TEST(SegmentsIntersect, Disjoint) {
  EXPECT_FALSE(gfg::segments_intersect({0, 0}, {1, 1}, {2, 2.5}, {3, 0}));
}

TEST(SegmentsIntersect, TouchingEndpointCounts) {
  EXPECT_TRUE(gfg::segments_intersect({0, 0}, {5, 5}, {5, 5}, {10, 0}));
}

TEST(SegmentsIntersect, TIntersectionCounts) {
  EXPECT_TRUE(gfg::segments_intersect({0, 0}, {10, 0}, {5, 0}, {5, 5}));
}

TEST(SegmentsIntersect, CollinearOverlapCountsButCollinearGapDoesNot) {
  EXPECT_TRUE(gfg::segments_intersect({0, 0}, {6, 0}, {4, 0}, {10, 0}));
  EXPECT_FALSE(gfg::segments_intersect({0, 0}, {4, 0}, {6, 0}, {10, 0}));
}

TEST(DistanceSegmentSegment, ParallelSegments) {
  EXPECT_DOUBLE_EQ(gfg::distance_segment_segment({0, 0}, {10, 0}, {0, 3}, {10, 3}), 3.0);
}

TEST(DistanceSegmentSegment, IntersectingIsZero) {
  EXPECT_DOUBLE_EQ(gfg::distance_segment_segment({0, 0}, {10, 10}, {0, 10}, {10, 0}), 0.0);
}

TEST(DistanceToBoundary, PointNearestEdge) {
  EXPECT_DOUBLE_EQ(gfg::distance_to_boundary(Vec2{2, 5}, kSquare), 2.0);
}

TEST(DistanceToBoundary, SegmentPassingTheInnerCorner) {
  // Both ends deep inside the L; the line between them clips the notch.
  EXPECT_DOUBLE_EQ(gfg::distance_to_boundary(Vec2{5, 18}, Vec2{18, 5}, kEll), 0.0);
  // Pulled back so it passes the corner (10,10) at a distance.
  const double d = gfg::distance_to_boundary(Vec2{4, 12}, Vec2{12, 4}, kEll);
  EXPECT_NEAR(d, 4.0 / std::sqrt(2.0), 1e-9);
}

TEST(BoundaryCrossings, CountsEdges) {
  EXPECT_EQ(gfg::boundary_crossings({5, 5}, {5, 6}, kSquare), 0u);
  EXPECT_EQ(gfg::boundary_crossings({5, 5}, {15, 5}, kSquare), 1u);
  EXPECT_EQ(gfg::boundary_crossings({-5, 5}, {15, 5}, kSquare), 2u);
}

TEST(BoundaryCrossings, ThroughAVertexOverCounts) {
  EXPECT_EQ(gfg::boundary_crossings({5, 5}, {15, 15}, kSquare), 2u);
}

TEST(SignedArea, SignFollowsOrientation) {
  EXPECT_DOUBLE_EQ(gfg::signed_area(kSquare), 100.0);
  const Polygon cw(kSquare.rbegin(), kSquare.rend());
  EXPECT_DOUBLE_EQ(gfg::signed_area(cw), -100.0);
  EXPECT_DOUBLE_EQ(gfg::signed_area(kEll), 300.0);
}

TEST(IsSimple, AcceptsConvexAndNonConvex) {
  EXPECT_TRUE(gfg::is_simple(kSquare));
  EXPECT_TRUE(gfg::is_simple(kEll));
  EXPECT_TRUE(gfg::is_simple({{0, 0}, {1, 0}, {0, 1}}));
}

TEST(IsSimple, RejectsBowtie) {
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {10, 10}, {10, 0}, {0, 10}}));
}

TEST(IsSimple, RejectsTooFewVertices) {
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {1, 0}}));
}

TEST(IsSimple, RejectsRepeatedVertex) {
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {10, 0}, {10, 10}, {10, 10}, {0, 10}}));
  // A figure-eight pinched at one shared vertex.
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {5, 5}, {10, 0}, {10, 10}, {5, 5}, {0, 10}}));
}

TEST(IsSimple, RejectsSpike) {
  // Goes out to (20,5) and straight back along the same line.
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {10, 0}, {10, 5}, {20, 5}, {15, 5}, {10, 10}, {0, 10}}));
}

TEST(IsSimple, RejectsEdgeTouchingNonAdjacentVertex) {
  // Vertex (5,0) lies on the first edge, which it is not adjacent to.
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {10, 0}, {10, 10}, {5, 0}, {0, 10}}));
}

TEST(IsSimple, RejectsCollinearDegenerateTriangle) {
  EXPECT_FALSE(gfg::is_simple({{0, 0}, {5, 0}, {10, 0}}));
}
