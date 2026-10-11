#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "geofence_guard/guard.hpp"

using gfg::Reason;
using gfg::Vec3;

namespace {

// The L-shaped field from examples/field.fence: [0,300]x[0,250] with the
// quadrant x>120, y>120 removed, and a mast at [200,215]x[40,55].
gfg::Envelope field() {
  gfg::Envelope env;
  env.frame = "map";
  env.margin = 5.0;
  env.min_altitude = 2.0;
  env.max_altitude = 120.0;
  env.max_state_age = 0.5;
  env.max_leg = 300.0;
  env.fence = {{0, 0}, {300, 0}, {300, 120}, {120, 120}, {120, 250}, {0, 250}};
  env.exclusions = {{"mast", {{200, 40}, {215, 40}, {215, 55}, {200, 55}}}};
  return env;
}

class GuardTest : public ::testing::Test {
 protected:
  gfg::Guard guard{field()};

  gfg::Decision check(Vec3 from, Vec3 to) const {
    return guard.check({"map", 10.0, from}, {"map", 10.1, to});
  }
};

bool has(const gfg::Decision& d, Reason r) {
  return std::any_of(d.violations.begin(), d.violations.end(),
                     [r](const gfg::Violation& v) { return v.reason == r; });
}

std::string reasons(const gfg::Decision& d) {
  std::string s;
  for (const auto& v : d.violations) s += std::string(gfg::reason_code(v.reason)) + ": " + v.detail + "\n";
  return s;
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

TEST_F(GuardTest, AcceptsALegWellInside) {
  const auto d = check({20, 20, 30}, {20, 230, 30});
  EXPECT_TRUE(d.accepted) << reasons(d);
  EXPECT_FALSE(d.recovery);
  EXPECT_TRUE(d.violations.empty());
}

TEST_F(GuardTest, RefusesTargetOutsideFence) {
  const auto d = check({20, 20, 30}, {400, 20, 30});
  EXPECT_FALSE(d.accepted);
  EXPECT_TRUE(has(d, Reason::TargetOutsideFence)) << reasons(d);
  // The path obviously leaves too; saying so twice adds nothing.
  EXPECT_FALSE(has(d, Reason::PathLeavesFence));
}

TEST_F(GuardTest, RefusesTargetInsideTheMarginBand) {
  const auto d = check({20, 20, 30}, {297, 20, 30});
  EXPECT_TRUE(has(d, Reason::TargetOutsideFence)) << reasons(d);
  EXPECT_NE(d.violations[0].detail.find("3.0 m inside the fence"), std::string::npos);
}

TEST_F(GuardTest, AcceptsTargetExactlyAtTheMargin) {
  EXPECT_TRUE(check({20, 20, 30}, {295, 20, 30}).accepted);
}

TEST_F(GuardTest, RefusesTargetInTheNotch) {
  // Inside the L's bounding box, outside the L.
  EXPECT_TRUE(has(check({20, 20, 30}, {200, 200, 30}), Reason::TargetOutsideFence));
}

TEST_F(GuardTest, RefusesAShortcutAcrossTheNotch) {
  // Both ends are safe; the straight line between them is not. This is the
  // case a target-only check misses.
  const auto d = check({20, 230, 30}, {280, 100, 30});
  EXPECT_FALSE(d.accepted);
  ASSERT_EQ(d.violations.size(), 1u) << reasons(d);
  EXPECT_EQ(d.violations[0].reason, Reason::PathLeavesFence);
  EXPECT_FALSE(d.recovery);
}

TEST_F(GuardTest, RefusesAPathThatGrazesTheInnerCornerInsideTheMargin) {
  // Never leaves the fence, but passes about 1 m below the corner (120,120).
  const auto d = check({20, 122, 30}, {280, 114, 30});
  EXPECT_FALSE(d.recovery);
  EXPECT_TRUE(has(d, Reason::PathLeavesFence)) << reasons(d);
}

TEST_F(GuardTest, AltitudeLimits) {
  EXPECT_TRUE(has(check({20, 20, 30}, {20, 30, 1}), Reason::AltitudeOutOfRange));
  EXPECT_TRUE(has(check({20, 20, 30}, {20, 30, 121}), Reason::AltitudeOutOfRange));
  EXPECT_TRUE(check({20, 20, 30}, {20, 30, 2}).accepted);
  EXPECT_TRUE(check({20, 20, 30}, {20, 30, 120}).accepted);
}

TEST_F(GuardTest, ClimbingOutOfTheGroundIsAllowed) {
  // The vehicle sits below min altitude before takeoff; only the target is
  // held to the band, and the path between two altitudes is monotone.
  EXPECT_TRUE(check({20, 20, 0}, {20, 20, 30}).accepted);
}

TEST_F(GuardTest, LegLimit) {
  const auto d = check({10, 10, 30}, {10, 240, 30});  // 230 m, under 300
  EXPECT_TRUE(d.accepted) << reasons(d);
  gfg::Envelope env = field();
  env.max_leg = 100.0;
  const gfg::Guard tight(env);
  const auto refused = tight.check({"map", 0, {10, 10, 30}}, {"map", 0, {10, 200, 30}});
  EXPECT_TRUE(has(refused, Reason::LegTooLong)) << reasons(refused);
}

TEST_F(GuardTest, LegLimitCountsTheClimb) {
  gfg::Envelope env = field();
  env.max_leg = 50.0;
  const gfg::Guard tight(env);
  // 40 m across, 40 m up: 56.6 m.
  EXPECT_TRUE(has(tight.check({"map", 0, {20, 20, 10}}, {"map", 0, {60, 20, 50}}),
                  Reason::LegTooLong));
}

TEST_F(GuardTest, RefusesTargetInsideExclusion) {
  const auto d = check({180, 20, 30}, {207, 47, 30});
  EXPECT_TRUE(has(d, Reason::TargetInExclusion)) << reasons(d);
  EXPECT_NE(d.violations[0].detail.find("'mast'"), std::string::npos);
}

TEST_F(GuardTest, RefusesTargetInsideExclusionMargin) {
  EXPECT_TRUE(has(check({180, 20, 30}, {197, 47, 30}), Reason::TargetInExclusion));
}

TEST_F(GuardTest, RefusesPathThroughExclusion) {
  const auto d = check({180, 47, 30}, {240, 47, 30});
  EXPECT_FALSE(d.accepted);
  ASSERT_EQ(d.violations.size(), 1u) << reasons(d);
  EXPECT_EQ(d.violations[0].reason, Reason::PathEntersExclusion);
}

TEST_F(GuardTest, AcceptsPathPastExclusionOutsideMargin) {
  EXPECT_TRUE(check({180, 30, 30}, {240, 30, 30}).accepted);
}

TEST_F(GuardTest, ExclusionZonesApplyAtEveryAltitude) {
  // The envelope is a prism: flying "over" the mast at the ceiling is still refused.
  EXPECT_TRUE(has(check({180, 47, 120}, {240, 47, 120}), Reason::PathEntersExclusion));
}

TEST_F(GuardTest, ReportsEveryViolationNotJustTheFirst) {
  const auto d = check({20, 20, 30}, {400, 20, 500});
  EXPECT_TRUE(has(d, Reason::AltitudeOutOfRange));
  EXPECT_TRUE(has(d, Reason::LegTooLong));
  EXPECT_TRUE(has(d, Reason::TargetOutsideFence));
}

// --- inputs the guard cannot reason about -----------------------------------

TEST_F(GuardTest, RefusesNonFiniteTarget) {
  for (const double bad : {kNaN, kInf, -kInf}) {
    for (int axis = 0; axis < 3; ++axis) {
      Vec3 to{20, 30, 30};
      (axis == 0 ? to.x : axis == 1 ? to.y : to.z) = bad;
      const auto d = check({20, 20, 30}, to);
      ASSERT_EQ(d.violations.size(), 1u);
      EXPECT_EQ(d.violations[0].reason, Reason::NonFinite);
    }
  }
}

TEST_F(GuardTest, RefusesNonFiniteStateOrStamp) {
  EXPECT_TRUE(has(check({kNaN, 20, 30}, {20, 30, 30}), Reason::NonFinite));
  EXPECT_TRUE(has(guard.check({"map", kNaN, {20, 20, 30}}, {"map", 0, {20, 30, 30}}),
                  Reason::NonFinite));
}

TEST_F(GuardTest, RefusesFrameMismatch) {
  // A setpoint in the vehicle's body frame would be numerically plausible and
  // geometrically meaningless.
  const auto d = guard.check({"map", 10, {20, 20, 30}}, {"base_link", 10, {20, 30, 30}});
  ASSERT_EQ(d.violations.size(), 1u);
  EXPECT_EQ(d.violations[0].reason, Reason::FrameMismatch);
  EXPECT_TRUE(has(guard.check({"odom", 10, {20, 20, 30}}, {"map", 10, {20, 30, 30}}),
                  Reason::FrameMismatch));
}

TEST_F(GuardTest, RefusesStaleState) {
  const auto d = guard.check({"map", 10.0, {20, 20, 30}}, {"map", 10.6, {20, 30, 30}});
  ASSERT_EQ(d.violations.size(), 1u);
  EXPECT_EQ(d.violations[0].reason, Reason::StaleState);
  EXPECT_TRUE(guard.check({"map", 10.0, {20, 20, 30}}, {"map", 10.5, {20, 30, 30}}).accepted);
}

TEST_F(GuardTest, RefusesStateFromTheFuture) {
  // A clock jump is as disqualifying as a gap.
  EXPECT_TRUE(has(guard.check({"map", 11.0, {20, 20, 30}}, {"map", 10.0, {20, 30, 30}}),
                  Reason::StaleState));
}

// --- recovery ---------------------------------------------------------------

TEST_F(GuardTest, FromTheMarginBandMovingInwardIsAccepted) {
  // The vehicle drifted to 2 m from the east edge. Under the normal rule every
  // path from here is within the margin, so nothing could bring it back.
  const auto d = check({298, 50, 30}, {250, 50, 30});
  EXPECT_TRUE(d.accepted) << reasons(d);
  EXPECT_TRUE(d.recovery);
}

TEST_F(GuardTest, FromTheMarginBandMovingAlongTheEdgeIsAcceptedIfTargetIsSafe) {
  // The target is the thing that has to be safe; skimming the band on the
  // way back in is the cost of recovering at all.
  EXPECT_TRUE(check({298, 50, 30}, {290, 100, 30}).accepted);
}

TEST_F(GuardTest, FromTheMarginBandTheTargetMustStillBeSafe) {
  const auto d = check({298, 50, 30}, {299, 60, 30});
  EXPECT_FALSE(d.accepted);
  EXPECT_TRUE(d.recovery);
  EXPECT_TRUE(has(d, Reason::TargetOutsideFence));
}

TEST_F(GuardTest, FromOutsideReturningAcrossTheBoundaryOnceIsAccepted) {
  const auto d = check({310, 50, 30}, {250, 50, 30});
  EXPECT_TRUE(d.accepted) << reasons(d);
  EXPECT_TRUE(d.recovery);
}

TEST_F(GuardTest, FromTheNotchCrossingBackInOnceIsAccepted) {
  // Outside, in the notch; the line enters through the edge y=120 and stays in.
  const auto d = check({200, 190, 30}, {60, 60, 30});
  EXPECT_TRUE(d.recovery);
  EXPECT_TRUE(d.accepted) << reasons(d);
}

TEST_F(GuardTest, FromOutsideAPathThatLeavesAgainIsRefused) {
  // West of the field: in through the west arm, out across the notch, back in
  // over the south arm. The target is safe and the path is three crossings.
  const auto d = check({-10, 200, 30}, {290, 100, 30});
  EXPECT_TRUE(d.recovery);
  ASSERT_TRUE(has(d, Reason::PathLeavesFence)) << reasons(d);
  EXPECT_NE(reasons(d).find("3 times; at most 1 allowed"), std::string::npos) << reasons(d);
}

TEST_F(GuardTest, ThroughAVertexCountsAsTwoCrossings) {
  // y = x from the notch passes exactly through (120,120), touching two edges.
  // Refusing is conservative; the operator can aim one metre either side.
  EXPECT_TRUE(has(check({200, 200, 30}, {60, 60, 30}), Reason::PathLeavesFence));
}

TEST_F(GuardTest, FromInsideTheBandAPathThatExitsIsRefused) {
  // 3 m from the inner corner's east edge, inside the fence, heading across
  // the notch to the far arm.
  const auto d = check({117, 200, 30}, {290, 100, 30});
  EXPECT_TRUE(d.recovery);
  EXPECT_TRUE(has(d, Reason::PathLeavesFence)) << reasons(d);
}

TEST_F(GuardTest, InsideAnExclusionZoneTheWayOutIsAllowed) {
  const auto d = check({207, 47, 30}, {180, 47, 30});
  EXPECT_TRUE(d.accepted) << reasons(d);
  EXPECT_TRUE(d.recovery);
}

TEST_F(GuardTest, InsideAnExclusionZoneTheWayThroughIsAllowedOnlyIfItExitsOnce) {
  // Exiting the far side is still one crossing.
  EXPECT_TRUE(check({207, 47, 30}, {240, 47, 30}).accepted);
}

TEST_F(GuardTest, NearAnExclusionZoneAPathIntoItIsRefused) {
  // In the mast's margin band, outside it; any crossing is too many.
  const auto d = check({198, 47, 30}, {230, 47, 30});
  EXPECT_TRUE(d.recovery);
  EXPECT_TRUE(has(d, Reason::PathEntersExclusion)) << reasons(d);
}

TEST(Guard, ConstructorValidates) {
  gfg::Envelope env = field();
  env.margin = 0.0;
  EXPECT_THROW(gfg::Guard{env}, gfg::EnvelopeError);
}

TEST(Guard, ReasonCodesAreStable) {
  // These strings are the ROS diagnostics interface; renaming one is a breaking change.
  EXPECT_STREQ(gfg::reason_code(Reason::PathLeavesFence), "path_leaves_fence");
  EXPECT_STREQ(gfg::reason_code(Reason::StaleState), "stale_state");
  EXPECT_STREQ(gfg::reason_code(Reason::TargetInExclusion), "target_in_exclusion");
}
