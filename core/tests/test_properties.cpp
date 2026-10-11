// The guard's answers checked against brute force. For thousands of random
// fences, exclusion zones and moves, walk the straight path in small steps and
// compare what the guard decided with what actually happens along the line.
//
// Soundness: whatever the guard accepts, the walk confirms.
// Usefulness: whatever the walk shows is comfortably safe, the guard accepts.
// A guard that refused everything would pass the first and fail the second.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "geofence_guard/guard.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSamples = 2000;
constexpr int kCases = 20000;

struct World {
  gfg::Envelope env;
  gfg::Vec3 from;
  gfg::Vec3 to;
};

// Star-shaped around its centre with strictly increasing angles, so always
// simple — and usually not convex, which is the interesting case.
gfg::Polygon star(std::mt19937_64& rng, gfg::Vec2 centre, double r_min, double r_max) {
  std::uniform_int_distribution<int> count(3, 12);
  std::uniform_real_distribution<double> radius(r_min, r_max);
  const int n = count(rng);
  std::uniform_real_distribution<double> jitter(0.1, 0.9);
  gfg::Polygon poly;
  for (int i = 0; i < n; ++i) {
    const double angle = 2.0 * kPi * (i + jitter(rng)) / n;
    const double r = radius(rng);
    poly.push_back({centre.x + r * std::cos(angle), centre.y + r * std::sin(angle)});
  }
  return poly;
}

World random_world(std::mt19937_64& rng) {
  std::uniform_real_distribution<double> coord(-250.0, 250.0);
  std::uniform_real_distribution<double> margin(1.0, 10.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  World w;
  w.env.frame = "map";
  w.env.margin = margin(rng);
  w.env.min_altitude = 0.0;
  w.env.max_altitude = 1000.0;
  w.env.max_state_age = 1.0;
  w.env.fence = star(rng, {0, 0}, 50.0, 220.0);
  if (unit(rng) < 0.7) {
    w.env.exclusions.push_back({"z", star(rng, {coord(rng) * 0.5, coord(rng) * 0.5}, 5.0, 40.0)});
  }
  // Mostly short moves starting near the middle, so a useful share of cases
  // are safe; the rest still range over the whole fence and beyond.
  std::uniform_real_distribution<double> step(-120.0, 120.0);
  w.from = {coord(rng) * 0.6, coord(rng) * 0.6, 30.0};
  w.to = unit(rng) < 0.8 ? gfg::Vec3{w.from.x + step(rng), w.from.y + step(rng), 30.0}
                         : gfg::Vec3{coord(rng), coord(rng), 30.0};
  return w;
}

// Distance from the boundary, positive on the safe side and negative on the
// wrong side.
double clearance(gfg::Vec2 p, const gfg::Polygon& poly, bool keep_in) {
  const double d = gfg::distance_to_boundary(p, poly);
  return gfg::point_in_polygon(p, poly) == keep_in ? d : -d;
}

gfg::Vec2 along(const World& w, int i) {
  const double t = static_cast<double>(i) / (kSamples - 1);
  return {w.from.x + t * (w.to.x - w.from.x), w.from.y + t * (w.to.y - w.from.y)};
}

// Smallest clearance seen along the walk for one region.
double walk_min_clearance(const World& w, const gfg::Polygon& poly, bool keep_in) {
  double best = INFINITY;
  for (int i = 0; i < kSamples; ++i) best = std::min(best, clearance(along(w, i), poly, keep_in));
  return best;
}

// Once the walk is on the safe side, does it ever go back to the wrong side?
bool walk_leaves_after_entering(const World& w, const gfg::Polygon& poly, bool keep_in) {
  bool entered = false;
  for (int i = 0; i < kSamples; ++i) {
    const bool safe_side = clearance(along(w, i), poly, keep_in) > 0.0;
    if (safe_side) entered = true;
    if (entered && !safe_side) return true;
  }
  return false;
}

struct Region {
  const gfg::Polygon* poly;
  bool keep_in;
};

std::vector<Region> regions(const World& w) {
  std::vector<Region> out{{&w.env.fence, true}};
  for (const auto& z : w.env.exclusions) out.push_back({&z.polygon, false});
  return out;
}

}  // namespace

TEST(Properties, AcceptedPathsAreSafeAlongTheirWholeLength) {
  std::mt19937_64 rng(20261011);
  int accepted_normal = 0;
  int accepted_recovery = 0;
  int refused = 0;
  const double eps = 1e-9;

  for (int c = 0; c < kCases; ++c) {
    const World w = random_world(rng);
    const gfg::Guard guard(w.env);
    const auto d = guard.check({"map", 0, w.from}, {"map", 0, w.to});
    if (!d.accepted) {
      ++refused;
      continue;
    }
    for (const auto& r : regions(w)) {
      const double target = clearance(w.to.xy(), *r.poly, r.keep_in);
      ASSERT_GE(target, w.env.margin - eps) << "case " << c << ": accepted an unsafe target";
      if (!d.recovery) {
        ASSERT_GE(walk_min_clearance(w, *r.poly, r.keep_in), w.env.margin - eps)
            << "case " << c << ": accepted a path that enters the margin";
      } else {
        ASSERT_FALSE(walk_leaves_after_entering(w, *r.poly, r.keep_in))
            << "case " << c << ": accepted a recovery path that leaves again";
      }
    }
    ++(d.recovery ? accepted_recovery : accepted_normal);
  }

  // Make sure the generator exercised every branch, not just refusals.
  EXPECT_GT(accepted_normal, kCases / 20);
  EXPECT_GT(accepted_recovery, kCases / 100);
  EXPECT_GT(refused, kCases / 20);
}

TEST(Properties, ComfortablySafePathsAreAccepted) {
  // Sample spacing is under 0.4 m and clearance changes at most 1 m per metre
  // travelled, so a walk that never comes within margin + 1 m of a boundary
  // proves the true path does not come within margin either.
  std::mt19937_64 rng(7);
  const double slack = 1.0;
  int checked = 0;

  for (int c = 0; c < kCases; ++c) {
    const World w = random_world(rng);
    bool comfortable = true;
    // Cheap endpoint test first; the walk is only worth doing if both pass.
    for (const auto& r : regions(w)) {
      if (clearance(w.from.xy(), *r.poly, r.keep_in) < w.env.margin + slack ||
          clearance(w.to.xy(), *r.poly, r.keep_in) < w.env.margin + slack) {
        comfortable = false;
      }
    }
    for (const auto& r : regions(w)) {
      if (!comfortable) break;
      if (walk_min_clearance(w, *r.poly, r.keep_in) < w.env.margin + slack) {
        comfortable = false;
        break;
      }
    }
    if (!comfortable) continue;
    ++checked;
    const gfg::Guard guard(w.env);
    const auto d = guard.check({"map", 0, w.from}, {"map", 0, w.to});
    ASSERT_TRUE(d.accepted) << "case " << c << ": refused a path with clearance to spare: "
                            << (d.violations.empty() ? "" : d.violations[0].detail);
    ASSERT_FALSE(d.recovery) << "case " << c;
  }
  EXPECT_GT(checked, kCases / 20);
}

TEST(Properties, RecoveryNeverAcceptsWhatNormalModeWouldRefuseFromASafeStart) {
  // Recovery is a different rule, so it must only ever apply when the vehicle
  // genuinely is outside the safe region.
  std::mt19937_64 rng(99);
  for (int c = 0; c < kCases; ++c) {
    const World w = random_world(rng);
    const gfg::Guard guard(w.env);
    const auto d = guard.check({"map", 0, w.from}, {"map", 0, w.to});
    bool start_safe = true;
    for (const auto& r : regions(w)) {
      if (clearance(w.from.xy(), *r.poly, r.keep_in) < w.env.margin) start_safe = false;
    }
    ASSERT_EQ(d.recovery, !start_safe) << "case " << c;
  }
}
