// The flight envelope: where the vehicle is permitted to be. It is loaded once,
// validated completely, and immutable afterwards. A file that is ambiguous in
// any way — an unknown key, a repeated key, a polygon that crosses itself — is
// refused rather than interpreted, because a misread limit is a limit that is
// not enforced.
#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geofence_guard/geometry.hpp"

namespace gfg {

struct ExclusionZone {
  std::string name;
  Polygon polygon;
};

struct Envelope {
  std::string frame;                   // setpoints in any other frame are refused
  double margin = 0.0;                 // metres kept from every boundary
  double min_altitude = 0.0;           // metres, frame z axis up
  double max_altitude = 0.0;
  double max_state_age = 0.0;          // seconds; older vehicle state means "unknown"
  std::optional<double> max_leg;       // metres between current position and target
  Polygon fence;                       // keep-in
  std::vector<ExclusionZone> exclusions;  // keep-out
};

class EnvelopeError : public std::runtime_error {
 public:
  EnvelopeError(int line, const std::string& message);
  int line() const { return line_; }  // 0 when the error is not tied to a line

 private:
  int line_;
};

Envelope parse_envelope(const std::string& text);
Envelope load_envelope(const std::string& path);

// Checks the invariants parse_envelope establishes; exposed so an envelope
// built in code is held to the same rules as one read from a file.
void validate_envelope(const Envelope& env);

}  // namespace gfg
