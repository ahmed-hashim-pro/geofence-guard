#include <gtest/gtest.h>

#include <string>

#include "geofence_guard/envelope.hpp"

namespace {

const std::string kValid = R"(
# comment line
frame map
margin 5
altitude 2 120
max_state_age 0.5
max_leg 300   # trailing comment

fence
  0 0
  100 0
  100 100
  0 100
end

exclude mast
  40 40
  50 40
  50 50
end
)";

// Replace the first occurrence of `from` in the valid file.
std::string with(const std::string& from, const std::string& to) {
  std::string s = kValid;
  const auto pos = s.find(from);
  EXPECT_NE(pos, std::string::npos) << from;
  return s.replace(pos, from.size(), to);
}

// Asserts parsing fails, and that the message mentions `needle`.
void expect_refused(const std::string& text, const std::string& needle) {
  try {
    gfg::parse_envelope(text);
    ADD_FAILURE() << "parsed, expected an error mentioning '" << needle << "'";
  } catch (const gfg::EnvelopeError& e) {
    EXPECT_NE(std::string(e.what()).find(needle), std::string::npos)
        << "message was: " << e.what();
  }
}

}  // namespace

TEST(Envelope, ParsesAValidFile) {
  const auto env = gfg::parse_envelope(kValid);
  EXPECT_EQ(env.frame, "map");
  EXPECT_DOUBLE_EQ(env.margin, 5.0);
  EXPECT_DOUBLE_EQ(env.min_altitude, 2.0);
  EXPECT_DOUBLE_EQ(env.max_altitude, 120.0);
  EXPECT_DOUBLE_EQ(env.max_state_age, 0.5);
  ASSERT_TRUE(env.max_leg.has_value());
  EXPECT_DOUBLE_EQ(*env.max_leg, 300.0);
  EXPECT_EQ(env.fence.size(), 4u);
  ASSERT_EQ(env.exclusions.size(), 1u);
  EXPECT_EQ(env.exclusions[0].name, "mast");
  EXPECT_EQ(env.exclusions[0].polygon.size(), 3u);
}

TEST(Envelope, MaxLegIsOptional) {
  const auto env = gfg::parse_envelope(with("max_leg 300", ""));
  EXPECT_FALSE(env.max_leg.has_value());
}

TEST(Envelope, ExclusionsAreOptional) {
  const std::string text = kValid.substr(0, kValid.find("exclude"));
  EXPECT_TRUE(gfg::parse_envelope(text).exclusions.empty());
}

TEST(Envelope, UnknownKeyIsAnError) {
  // A misspelt limit that was silently ignored would be a limit not enforced.
  expect_refused(with("max_leg 300", "max_alitude 60"), "unknown key 'max_alitude'");
}

TEST(Envelope, ErrorsCarryTheLineNumber) {
  try {
    gfg::parse_envelope(with("max_leg 300", "bogus 1"));
    FAIL();
  } catch (const gfg::EnvelopeError& e) {
    EXPECT_EQ(e.line(), 7);
  }
}

TEST(Envelope, RepeatedKeyIsAnError) {
  expect_refused(with("margin 5", "margin 5\nmargin 1"), "'margin' is set twice");
}

TEST(Envelope, SecondFenceIsAnError) {
  expect_refused(kValid + "fence\n0 0\n1 0\n0 1\nend\n", "'fence' is set twice");
}

TEST(Envelope, MissingRequiredKeys) {
  expect_refused(with("frame map", ""), "missing required 'frame'");
  expect_refused(with("margin 5", ""), "missing required 'margin'");
  expect_refused(with("altitude 2 120", ""), "missing required 'altitude'");
  expect_refused(with("max_state_age 0.5", ""), "missing required 'max_state_age'");
  expect_refused(R"(frame map
margin 5
altitude 2 120
max_state_age 0.5
)",
                 "missing required 'fence'");
}

TEST(Envelope, WrongArity) {
  expect_refused(with("altitude 2 120", "altitude 120"), "'altitude' takes 2 values, got 1");
  expect_refused(with("margin 5", "margin 5 6"), "'margin' takes 1 value, got 2");
  expect_refused(with("  100 0\n", "  100 0 7\n"), "expected a vertex");
}

TEST(Envelope, NumbersMustBeWholeAndFinite) {
  expect_refused(with("margin 5", "margin 5m"), "'5m' is not a finite number");
  expect_refused(with("margin 5", "margin nan"), "not a finite number");
  expect_refused(with("margin 5", "margin inf"), "not a finite number");
  expect_refused(with("margin 5", "margin 5,0"), "not a finite number");
  expect_refused(with("  100 0\n", "  1e999 0\n"), "not a finite number");
}

TEST(Envelope, NumbersParseRegardlessOfLocale) {
  // from_chars ignores the C locale, so this holds under de_DE as well.
  EXPECT_DOUBLE_EQ(gfg::parse_envelope(with("margin 5", "margin 2.5")).margin, 2.5);
}

TEST(Envelope, MarginMustBePositive) {
  expect_refused(with("margin 5", "margin 0"), "margin must be greater than zero");
  expect_refused(with("margin 5", "margin -1"), "margin must be greater than zero");
}

TEST(Envelope, AltitudeBandMustBeOrdered) {
  expect_refused(with("altitude 2 120", "altitude 120 2"), "altitude minimum must be below");
  expect_refused(with("altitude 2 120", "altitude 50 50"), "altitude minimum must be below");
}

TEST(Envelope, StateAgeAndLegMustBePositive) {
  expect_refused(with("max_state_age 0.5", "max_state_age 0"), "max_state_age");
  expect_refused(with("max_leg 300", "max_leg -3"), "max_leg");
}

TEST(Envelope, FenceMustBeSimple) {
  expect_refused(with("  100 100\n  0 100\n", "  0 100\n  100 100\n"), "fence is not a simple polygon");
}

TEST(Envelope, FenceNeedsThreeVertices) {
  expect_refused(with("  100 100\n  0 100\n", ""), "fence needs at least 3 vertices, got 2");
}

TEST(Envelope, UnclosedPolygon) {
  expect_refused(kValid.substr(0, kValid.rfind("end")), "exclusion zone 'mast' is never closed");
}

TEST(Envelope, ExclusionZonesNeedUniqueSafeNames) {
  expect_refused(kValid + "exclude mast\n1 1\n2 1\n1 2\nend\n", "defined twice");
  expect_refused(with("exclude mast", "exclude ma%st"), "must be letters, digits");
  expect_refused(with("exclude mast", "exclude"), "'exclude' takes 1 value, got 0");
}

TEST(Envelope, FrameMustBeAnIdentifier) {
  expect_refused(with("frame map", "frame ma/p"), "frame must be letters");
}

TEST(Envelope, LoadReportsMissingFile) {
  EXPECT_THROW(gfg::load_envelope("/nonexistent/field.fence"), gfg::EnvelopeError);
}

TEST(Envelope, ValidateHoldsCodeBuiltEnvelopesToTheSameRules) {
  gfg::Envelope env = gfg::parse_envelope(kValid);
  env.fence = {{0, 0}, {10, 10}, {10, 0}, {0, 10}};
  EXPECT_THROW(gfg::validate_envelope(env), gfg::EnvelopeError);
}
