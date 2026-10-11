// geofence-guard: check an envelope file, or check a mission against one
// before it flies. The same Guard the ROS 2 node runs makes every decision
// here, so a mission that passes this has passed the in-flight rules leg by
// leg, under the assumption that the vehicle reaches each waypoint.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "geofence_guard/envelope.hpp"
#include "geofence_guard/guard.hpp"
#include "geofence_guard/version.hpp"
#include "render.hpp"

namespace {

constexpr int kRefused = 1;
constexpr int kUsage = 2;

void usage() {
  std::cerr << "usage:\n"
               "  geofence-guard validate <envelope>\n"
               "  geofence-guard check <envelope> <mission.csv>\n"
               "  geofence-guard render <envelope> <mission.csv>   (SVG to stdout)\n"
               "  geofence-guard version\n"
               "\n"
               "mission.csv: one 'x,y,z' per line in the envelope's frame, metres.\n"
               "The first line is where the vehicle starts; each later line is a\n"
               "waypoint, checked as a leg from the one before it.\n";
}

std::string point(const gfg::Vec3& p) {
  char buf[96];
  std::snprintf(buf, sizeof buf, "(%.1f, %.1f, %.1f)", p.x, p.y, p.z);
  return buf;
}

std::vector<gfg::Vec3> load_mission(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open mission file '" + path + "'");
  std::vector<gfg::Vec3> points;
  std::string raw;
  int line = 0;
  while (std::getline(file, raw)) {
    ++line;
    const std::string code = raw.substr(0, raw.find('#'));
    if (code.find_first_not_of(" \t\r") == std::string::npos) continue;
    std::istringstream in(code);
    gfg::Vec3 p;
    char c1 = 0;
    char c2 = 0;
    std::string rest;
    if (!(in >> p.x >> c1 >> p.y >> c2 >> p.z) || c1 != ',' || c2 != ',' || (in >> rest)) {
      throw std::runtime_error(path + ": line " + std::to_string(line) +
                               ": expected 'x,y,z', got '" + code + "'");
    }
    points.push_back(p);
  }
  if (points.size() < 2) {
    throw std::runtime_error(path + ": needs a start position and at least one waypoint");
  }
  return points;
}

void describe(const std::string& path, const gfg::Envelope& env) {
  std::printf("envelope   %s\n", path.c_str());
  std::printf("frame      %s\n", env.frame.c_str());
  std::printf("margin     %.1f m\n", env.margin);
  std::printf("altitude   %.1f to %.1f m\n", env.min_altitude, env.max_altitude);
  std::printf("state age  %.2f s max\n", env.max_state_age);
  if (env.max_leg) {
    std::printf("max leg    %.1f m\n", *env.max_leg);
  } else {
    std::printf("max leg    unlimited\n");
  }
  std::printf("fence      %zu vertices, %.0f m^2\n", env.fence.size(),
              std::abs(gfg::signed_area(env.fence)));
  for (const auto& z : env.exclusions) {
    std::printf("exclusion  %s, %zu vertices, %.0f m^2\n", z.name.c_str(), z.polygon.size(),
                std::abs(gfg::signed_area(z.polygon)));
  }
}

std::string basename(const std::string& path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

int check(const std::string& envelope_path, const std::string& mission_path) {
  const gfg::Guard guard(gfg::load_envelope(envelope_path));
  const auto legs = gfg_cli::decide_legs(guard, load_mission(mission_path));

  std::size_t ok = 0;
  for (std::size_t i = 0; i < legs.size(); ++i) {
    const auto& d = legs[i].decision;
    std::printf("leg %-3zu %s -> %s  %s%s\n", i + 1, point(legs[i].from).c_str(),
                point(legs[i].to).c_str(), d.accepted ? "ok" : "REFUSED",
                d.recovery ? "  (recovery)" : "");
    for (const auto& v : d.violations) {
      std::printf("        %s: %s\n", gfg::reason_code(v.reason), v.detail.c_str());
    }
    if (d.accepted) ++ok;
  }
  std::printf("\n%zu/%zu legs ok\n", ok, legs.size());
  return ok == legs.size() ? 0 : kRefused;
}

int render(const std::string& envelope_path, const std::string& mission_path) {
  const gfg::Guard guard(gfg::load_envelope(envelope_path));
  const auto legs = gfg_cli::decide_legs(guard, load_mission(mission_path));
  const auto ok = std::count_if(legs.begin(), legs.end(),
                                [](const gfg_cli::Leg& l) { return l.decision.accepted; });
  const std::string title = basename(envelope_path) + " + " + basename(mission_path) + ": " +
                            std::to_string(ok) + "/" + std::to_string(legs.size()) + " legs ok";
  std::fputs(gfg_cli::render_svg(guard.envelope(), legs, title).c_str(), stdout);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  try {
    if (args.size() == 1 && args[0] == "version") {
      std::printf("geofence-guard %s\n", gfg::kVersion);
      return 0;
    }
    if (args.size() == 2 && args[0] == "validate") {
      describe(args[1], gfg::load_envelope(args[1]));
      return 0;
    }
    if (args.size() == 3 && args[0] == "check") return check(args[1], args[2]);
    if (args.size() == 3 && args[0] == "render") return render(args[1], args[2]);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return kUsage;
  }
  usage();
  return kUsage;
}
