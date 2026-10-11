// Planar geometry for fence checks. Everything is in metres in the local
// horizontal frame; altitude is handled separately because the envelope is a
// prism (a polygon extruded between two altitudes), not a general 3-D solid.
#pragma once

#include <cstddef>
#include <vector>

namespace gfg {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec2 xy() const { return {x, y}; }
};

using Polygon = std::vector<Vec2>;

// Ray-casting containment. A point exactly on an edge may land either way,
// which is why every caller also demands a distance from the boundary: the
// margin is what makes the answer stable, not this function.
bool point_in_polygon(Vec2 p, const Polygon& poly);

double distance_point_segment(Vec2 p, Vec2 a, Vec2 b);

// Closed segments: touching at an endpoint or overlapping collinearly counts.
bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d);

double distance_segment_segment(Vec2 a, Vec2 b, Vec2 c, Vec2 d);

// Shortest distance from a point, or from any point of a segment, to the
// polygon's boundary.
double distance_to_boundary(Vec2 p, const Polygon& poly);
double distance_to_boundary(Vec2 a, Vec2 b, const Polygon& poly);

// How many polygon edges the segment touches. A segment passing exactly
// through a vertex touches two edges, so this over-counts rather than
// under-counts — the safe direction for a guard.
std::size_t boundary_crossings(Vec2 a, Vec2 b, const Polygon& poly);

double signed_area(const Polygon& poly);

// No repeated vertices, no zero-length edges, no edge touching any edge it is
// not adjacent to, and adjacent edges meeting only at their shared vertex.
bool is_simple(const Polygon& poly);

}  // namespace gfg
