// The decision: given where the vehicle is and where something wants to send
// it, is the move provably inside the envelope? The guard is stateless — every
// input it needs is an argument — so a decision can be replayed exactly from a
// log line.
#pragma once

#include <string>
#include <vector>

#include "geofence_guard/envelope.hpp"
#include "geofence_guard/geometry.hpp"

namespace gfg {

struct VehicleState {
  std::string frame;
  double stamp = 0.0;  // seconds, same clock as Setpoint::stamp
  Vec3 position;
};

struct Setpoint {
  std::string frame;
  double stamp = 0.0;
  Vec3 position;
};

enum class Reason {
  NonFinite,           // a coordinate or timestamp is NaN or infinite
  FrameMismatch,       // state or setpoint is not in the envelope's frame
  StaleState,          // the vehicle's position is too old to reason from
  AltitudeOutOfRange,
  LegTooLong,
  TargetOutsideFence,
  PathLeavesFence,
  TargetInExclusion,
  PathEntersExclusion,
};

const char* reason_code(Reason r);

struct Violation {
  Reason reason;
  std::string detail;
};

struct Decision {
  bool accepted = false;
  // The vehicle was not inside the safe region when this was decided, so the
  // path rules for leaving the margin band applied. See README "Recovery".
  bool recovery = false;
  std::vector<Violation> violations;
};

class Guard {
 public:
  // Validates the envelope; throws EnvelopeError if it is not usable.
  explicit Guard(Envelope envelope);

  Decision check(const VehicleState& state, const Setpoint& setpoint) const;

  const Envelope& envelope() const { return env_; }

 private:
  Envelope env_;
};

}  // namespace gfg
