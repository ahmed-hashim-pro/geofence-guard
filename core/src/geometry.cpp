#include "geofence_guard/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gfg {

namespace {

double cross(Vec2 o, Vec2 a, Vec2 b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

int orientation(Vec2 o, Vec2 a, Vec2 b) {
  const double c = cross(o, a, b);
  if (c > 0.0) return 1;
  if (c < 0.0) return -1;
  return 0;
}

// p is known to be collinear with a-b; is it within the segment's box?
bool on_segment(Vec2 p, Vec2 a, Vec2 b) {
  return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

Vec2 vertex(const Polygon& poly, std::size_t i) { return poly[i % poly.size()]; }

}  // namespace

bool point_in_polygon(Vec2 p, const Polygon& poly) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Vec2 a = poly[i];
    const Vec2 b = poly[j];
    if ((a.y > p.y) != (b.y > p.y)) {
      const double x_at = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < x_at) inside = !inside;
    }
  }
  return inside;
}

double distance_point_segment(Vec2 p, Vec2 a, Vec2 b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) {
    t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = std::clamp(t, 0.0, 1.0);
  }
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  const int o1 = orientation(a, b, c);
  const int o2 = orientation(a, b, d);
  const int o3 = orientation(c, d, a);
  const int o4 = orientation(c, d, b);

  if (o1 != o2 && o3 != o4) return true;
  if (o1 == 0 && on_segment(c, a, b)) return true;
  if (o2 == 0 && on_segment(d, a, b)) return true;
  if (o3 == 0 && on_segment(a, c, d)) return true;
  if (o4 == 0 && on_segment(b, c, d)) return true;
  return false;
}

double distance_segment_segment(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  if (segments_intersect(a, b, c, d)) return 0.0;
  return std::min({distance_point_segment(a, c, d), distance_point_segment(b, c, d),
                   distance_point_segment(c, a, b), distance_point_segment(d, a, b)});
}

double distance_to_boundary(Vec2 p, const Polygon& poly) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, distance_point_segment(p, vertex(poly, i), vertex(poly, i + 1)));
  }
  return best;
}

double distance_to_boundary(Vec2 a, Vec2 b, const Polygon& poly) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, distance_segment_segment(a, b, vertex(poly, i), vertex(poly, i + 1)));
  }
  return best;
}

std::size_t boundary_crossings(Vec2 a, Vec2 b, const Polygon& poly) {
  std::size_t count = 0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    if (segments_intersect(a, b, vertex(poly, i), vertex(poly, i + 1))) ++count;
  }
  return count;
}

double signed_area(const Polygon& poly) {
  double twice = 0.0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Vec2 p = vertex(poly, i);
    const Vec2 q = vertex(poly, i + 1);
    twice += p.x * q.y - q.x * p.y;
  }
  return twice / 2.0;
}

bool is_simple(const Polygon& poly) {
  const std::size_t n = poly.size();
  if (n < 3) return false;

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      if (poly[i].x == poly[j].x && poly[i].y == poly[j].y) return false;
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = vertex(poly, i);
    const Vec2 b = vertex(poly, i + 1);
    for (std::size_t j = i + 1; j < n; ++j) {
      const Vec2 c = vertex(poly, j);
      const Vec2 d = vertex(poly, j + 1);
      const bool next = (j == i + 1);
      const bool wraps = (i == 0 && j == n - 1);
      if (next) {
        // Edges a-b and b-d share b. They must not fold back over each other.
        if (orientation(a, b, d) == 0 && on_segment(d, a, b)) return false;
        if (orientation(b, d, a) == 0 && on_segment(a, b, d)) return false;
      } else if (wraps) {
        // Edges a-b and c-a share a.
        if (orientation(c, a, b) == 0 && on_segment(b, c, a)) return false;
        if (orientation(a, b, c) == 0 && on_segment(c, a, b)) return false;
      } else if (segments_intersect(a, b, c, d)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace gfg
