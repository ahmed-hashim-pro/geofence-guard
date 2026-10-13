#include "geofence_guard/envelope.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace gfg {

EnvelopeError::EnvelopeError(int line, const std::string& message)
    : std::runtime_error(line > 0 ? "line " + std::to_string(line) + ": " + message : message),
      line_(line) {}

namespace {

std::vector<std::string> tokenize(const std::string& line) {
  const std::string code = line.substr(0, line.find('#'));
  std::istringstream in(code);
  std::vector<std::string> tokens;
  for (std::string t; in >> t;) tokens.push_back(t);
  return tokens;
}

// from_chars rather than strtod: strtod honours the C locale, and a node
// started under a locale with a decimal comma would read "2.5" as 2.
double number(const std::string& token, int line) {
  double value = 0.0;
  const char* begin = token.data();
  const char* end = begin + token.size();
  const auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec != std::errc() || ptr != end || !std::isfinite(value)) {
    throw EnvelopeError(line, "'" + token + "' is not a finite number");
  }
  return value;
}

bool valid_name(const std::string& name) {
  if (name.empty() || name.size() > 64) return false;
  for (const char c : name) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

void expect_args(const std::vector<std::string>& t, std::size_t n, int line) {
  if (t.size() != n + 1) {
    throw EnvelopeError(line, "'" + t[0] + "' takes " + std::to_string(n) + " value" +
                                  (n == 1 ? "" : "s") + ", got " + std::to_string(t.size() - 1));
  }
}

void check_polygon(const Polygon& poly, const std::string& what, int line) {
  if (poly.size() < 3) {
    throw EnvelopeError(line, what + " needs at least 3 vertices, got " +
                                  std::to_string(poly.size()));
  }
  if (!is_simple(poly)) {
    throw EnvelopeError(line, what + " is not a simple polygon (an edge crosses or touches "
                                     "another, or a vertex repeats)");
  }
  if (std::abs(signed_area(poly)) <= 0.0) {
    throw EnvelopeError(line, what + " has zero area");
  }
}

// Where each directive appeared, so a value that parses but is not allowed
// (a zero margin, an inverted altitude band) is reported at its line like any
// other error. All zero for an envelope built in code.
struct DirectiveLines {
  int frame = 0;
  int margin = 0;
  int altitude = 0;
  int max_state_age = 0;
  int max_leg = 0;
  int fence = 0;
  std::vector<int> exclusions;  // parallel to Envelope::exclusions
};

void check_values(const Envelope& env, const DirectiveLines& at) {
  if (!valid_name(env.frame)) throw EnvelopeError(at.frame, "frame must be a plain identifier");
  if (!(env.margin > 0.0)) {
    throw EnvelopeError(at.margin, "margin must be greater than zero: a vehicle tracking a "
                                   "setpoint on the fence line is outside it half the time");
  }
  if (!(env.min_altitude < env.max_altitude)) {
    throw EnvelopeError(at.altitude, "altitude minimum must be below the maximum");
  }
  if (!(env.max_state_age > 0.0)) {
    throw EnvelopeError(at.max_state_age, "max_state_age must be greater than zero");
  }
  if (env.max_leg && !(*env.max_leg > 0.0)) {
    throw EnvelopeError(at.max_leg, "max_leg must be greater than zero");
  }
  check_polygon(env.fence, "fence", at.fence);

  std::set<std::string> names;
  for (std::size_t i = 0; i < env.exclusions.size(); ++i) {
    const auto& zone = env.exclusions[i];
    const int line = i < at.exclusions.size() ? at.exclusions[i] : 0;
    if (!valid_name(zone.name)) throw EnvelopeError(line, "exclusion zone name '" + zone.name + "' is not a plain identifier");
    if (!names.insert(zone.name).second) throw EnvelopeError(line, "exclusion zone '" + zone.name + "' is defined twice");
    check_polygon(zone.polygon, "exclusion zone '" + zone.name + "'", line);
  }
}

}  // namespace

void validate_envelope(const Envelope& env) { check_values(env, DirectiveLines{}); }

Envelope parse_envelope(const std::string& text) {
  Envelope env;
  DirectiveLines at;
  std::set<std::string> seen;
  std::istringstream in(text);

  Polygon* open_polygon = nullptr;
  std::string open_what;
  int open_line = 0;

  std::string raw;
  int line = 0;
  while (std::getline(in, raw)) {
    ++line;
    const auto t = tokenize(raw);
    if (t.empty()) continue;

    if (open_polygon != nullptr) {
      if (t[0] == "end") {
        if (t.size() != 1) throw EnvelopeError(line, "'end' takes no values");
        check_polygon(*open_polygon, open_what, open_line);
        open_polygon = nullptr;
        continue;
      }
      if (t.size() != 2) {
        throw EnvelopeError(line, "expected a vertex 'x y' or 'end' inside " + open_what);
      }
      open_polygon->push_back({number(t[0], line), number(t[1], line)});
      continue;
    }

    const std::string& key = t[0];
    if (key == "exclude") {
      expect_args(t, 1, line);
      if (!valid_name(t[1])) {
        throw EnvelopeError(line, "exclusion zone name '" + t[1] +
                                      "' must be letters, digits, '_' or '-'");
      }
      for (const auto& z : env.exclusions) {
        if (z.name == t[1]) throw EnvelopeError(line, "exclusion zone '" + t[1] + "' is defined twice");
      }
      env.exclusions.push_back({t[1], {}});
      at.exclusions.push_back(line);
      open_polygon = &env.exclusions.back().polygon;
      open_what = "exclusion zone '" + t[1] + "'";
      open_line = line;
      continue;
    }

    if (!seen.insert(key).second) throw EnvelopeError(line, "'" + key + "' is set twice");

    if (key == "frame") {
      at.frame = line;
      expect_args(t, 1, line);
      if (!valid_name(t[1])) throw EnvelopeError(line, "frame must be letters, digits, '_' or '-'");
      env.frame = t[1];
    } else if (key == "margin") {
      at.margin = line;
      expect_args(t, 1, line);
      env.margin = number(t[1], line);
    } else if (key == "altitude") {
      at.altitude = line;
      expect_args(t, 2, line);
      env.min_altitude = number(t[1], line);
      env.max_altitude = number(t[2], line);
    } else if (key == "max_state_age") {
      at.max_state_age = line;
      expect_args(t, 1, line);
      env.max_state_age = number(t[1], line);
    } else if (key == "max_leg") {
      at.max_leg = line;
      expect_args(t, 1, line);
      env.max_leg = number(t[1], line);
    } else if (key == "fence") {
      at.fence = line;
      expect_args(t, 0, line);
      open_polygon = &env.fence;
      open_what = "fence";
      open_line = line;
    } else {
      throw EnvelopeError(line, "unknown key '" + key + "'");
    }
  }

  if (open_polygon != nullptr) {
    throw EnvelopeError(open_line, open_what + " is never closed with 'end'");
  }
  for (const char* required : {"frame", "margin", "altitude", "max_state_age", "fence"}) {
    if (seen.count(required) == 0) throw EnvelopeError(0, std::string("missing required '") + required + "'");
  }

  check_values(env, at);
  return env;
}

Envelope load_envelope(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw EnvelopeError(0, "cannot open envelope file '" + path + "'");
  std::ostringstream content;
  content << file.rdbuf();
  try {
    return parse_envelope(content.str());
  } catch (const EnvelopeError& e) {
    throw EnvelopeError(0, path + ": " + e.what());
  }
}

}  // namespace gfg
