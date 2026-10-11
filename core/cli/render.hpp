// Draws an envelope and a mission as a standalone SVG: the fence, the margin
// band, exclusion zones, and every leg coloured by the Guard's decision on it.
// The picture is computed from the same decisions `check` prints, so it can
// be trusted the same amount.
#pragma once

#include <string>
#include <vector>

#include "geofence_guard/guard.hpp"

namespace gfg_cli {

struct Leg {
  gfg::Vec3 from;
  gfg::Vec3 to;
  gfg::Decision decision;
};

std::vector<Leg> decide_legs(const gfg::Guard& guard, const std::vector<gfg::Vec3>& mission);

std::string render_svg(const gfg::Envelope& env, const std::vector<Leg>& legs,
                       const std::string& title);

}  // namespace gfg_cli
