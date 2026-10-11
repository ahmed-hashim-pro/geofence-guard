#include "geofence_guard/guard.hpp"

#include <cmath>
#include <cstdio>

namespace gfg {

const char* reason_code(Reason r) {
  switch (r) {
    case Reason::NonFinite: return "non_finite";
    case Reason::FrameMismatch: return "frame_mismatch";
    case Reason::StaleState: return "stale_state";
    case Reason::AltitudeOutOfRange: return "altitude_out_of_range";
    case Reason::LegTooLong: return "leg_too_long";
    case Reason::TargetOutsideFence: return "target_outside_fence";
    case Reason::PathLeavesFence: return "path_leaves_fence";
    case Reason::TargetInExclusion: return "target_in_exclusion";
    case Reason::PathEntersExclusion: return "path_enters_exclusion";
  }
  return "unknown";
}

namespace {

template <typename... Args>
std::string fmt(const std::string& format, Args... args) {
  char buf[200];
  std::snprintf(buf, sizeof buf, format.c_str(), args...);
  return buf;
}

bool finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

struct RegionCheck {
  bool target_safe = false;
  bool path_safe = false;
  bool recovery = false;
  double target_clearance = 0.0;  // distance from target to boundary
  double path_clearance = 0.0;    // normal mode only
  std::size_t crossings = 0;      // recovery mode only
  std::size_t allowed_crossings = 0;
};

// One rule for both kinds of region; keep_in says which side is the safe one.
//
// Normal mode (the vehicle is at least `margin` inside the safe side): the
// whole straight path must stay at least `margin` from the boundary. Since
// the start is on the safe side and the path never gets within `margin` of
// the boundary, it cannot cross it — so the end is on the safe side too.
//
// Recovery mode (the vehicle has drifted into the margin band or beyond): the
// rule above would refuse every setpoint, including the one that brings it
// back. Instead the target must be safe and the path may cross the boundary
// only as often as it needs to get back: zero times from inside the band, once
// from the wrong side. A non-convex fence where the path would leave again is
// still refused.
RegionCheck check_region(Vec2 a, Vec2 b, const Polygon& poly, double margin, bool keep_in) {
  const auto right_side = [&](Vec2 p) { return point_in_polygon(p, poly) == keep_in; };

  RegionCheck r;
  r.target_clearance = distance_to_boundary(b, poly);
  r.target_safe = right_side(b) && r.target_clearance >= margin;
  r.recovery = !(right_side(a) && distance_to_boundary(a, poly) >= margin);
  if (!r.target_safe) return r;

  if (!r.recovery) {
    r.path_clearance = distance_to_boundary(a, b, poly);
    r.path_safe = r.path_clearance >= margin;
  } else {
    r.crossings = boundary_crossings(a, b, poly);
    r.allowed_crossings = right_side(a) ? 0 : 1;
    r.path_safe = r.crossings <= r.allowed_crossings;
  }
  return r;
}

std::string path_detail(const RegionCheck& r, const std::string& what, double margin) {
  if (!r.recovery) {
    if (r.path_clearance <= 0.0) return "path crosses the boundary of " + what;
    return fmt("path comes within %.1f m of " + what + "; margin is %.1f m",
               r.path_clearance, margin);
  }
  return "returning path crosses the boundary of " + what + " " + std::to_string(r.crossings) +
         " times; at most " + std::to_string(r.allowed_crossings) + " allowed";
}

}  // namespace

Guard::Guard(Envelope envelope) : env_(std::move(envelope)) { validate_envelope(env_); }

Decision Guard::check(const VehicleState& state, const Setpoint& setpoint) const {
  Decision d;
  const auto refuse = [&](Reason reason, std::string detail) {
    d.violations.push_back({reason, std::move(detail)});
  };

  // These three make every later check meaningless, so they end the decision.
  if (!finite(state.position) || !finite(setpoint.position) || !std::isfinite(state.stamp) ||
      !std::isfinite(setpoint.stamp)) {
    refuse(Reason::NonFinite, "a coordinate or timestamp is NaN or infinite");
    return d;
  }
  if (setpoint.frame != env_.frame || state.frame != env_.frame) {
    refuse(Reason::FrameMismatch, "envelope is in '" + env_.frame + "', setpoint in '" +
                                      setpoint.frame + "', vehicle state in '" + state.frame + "'");
    return d;
  }
  const double age = setpoint.stamp - state.stamp;
  if (std::abs(age) > env_.max_state_age) {
    refuse(Reason::StaleState,
           fmt("vehicle state is %.2f s from the setpoint; limit is %.2f s", age, env_.max_state_age));
    return d;
  }

  const Vec3 a3 = state.position;
  const Vec3 b3 = setpoint.position;
  const Vec2 a = a3.xy();
  const Vec2 b = b3.xy();

  if (b3.z < env_.min_altitude || b3.z > env_.max_altitude) {
    refuse(Reason::AltitudeOutOfRange,
           fmt("target altitude %.1f m is outside [%.1f, %.1f] m", b3.z, env_.min_altitude,
               env_.max_altitude));
  }

  if (env_.max_leg) {
    const double leg = std::hypot(b3.x - a3.x, b3.y - a3.y, b3.z - a3.z);
    if (leg > *env_.max_leg) {
      refuse(Reason::LegTooLong, fmt("leg is %.1f m; limit is %.1f m", leg, *env_.max_leg));
    }
  }

  const RegionCheck fence = check_region(a, b, env_.fence, env_.margin, true);
  d.recovery = d.recovery || fence.recovery;
  if (!fence.target_safe) {
    refuse(Reason::TargetOutsideFence,
           point_in_polygon(b, env_.fence)
               ? fmt("target is %.1f m inside the fence; margin is %.1f m", fence.target_clearance,
                     env_.margin)
               : fmt("target is %.1f m outside the fence", fence.target_clearance));
  } else if (!fence.path_safe) {
    refuse(Reason::PathLeavesFence, path_detail(fence, "the fence", env_.margin));
  }

  for (const auto& zone : env_.exclusions) {
    const RegionCheck z = check_region(a, b, zone.polygon, env_.margin, false);
    d.recovery = d.recovery || z.recovery;
    const std::string what = "exclusion zone '" + zone.name + "'";
    if (!z.target_safe) {
      refuse(Reason::TargetInExclusion,
             point_in_polygon(b, zone.polygon)
                 ? "target is inside " + what
                 : fmt("target is %.1f m from " + what + "; margin is %.1f m",
                       z.target_clearance, env_.margin));
    } else if (!z.path_safe) {
      refuse(Reason::PathEntersExclusion, path_detail(z, what, env_.margin));
    }
  }

  d.accepted = d.violations.empty();
  return d;
}

}  // namespace gfg
