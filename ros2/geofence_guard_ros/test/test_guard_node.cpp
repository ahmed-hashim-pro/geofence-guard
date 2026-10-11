// The node driven over real topics, in one process: a test node publishes
// odometry and setpoints and records what comes out the other side.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "geofence_guard_ros/guard_node.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

namespace {

class GuardNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { start({}); }

  void start(std::vector<rclcpp::Parameter> extra) {
    std::vector<rclcpp::Parameter> params{{"envelope_file", std::string(TEST_ENVELOPE)}};
    params.insert(params.end(), extra.begin(), extra.end());
    guard = std::make_shared<geofence_guard_ros::GuardNode>(
        rclcpp::NodeOptions().parameter_overrides(params));

    harness = std::make_shared<rclcpp::Node>("harness");
    odom_pub = harness->create_publisher<Odometry>("odometry", 10);
    sp_pub = harness->create_publisher<PoseStamped>("setpoint_in", 10);
    out_sub = harness->create_subscription<PoseStamped>(
        "setpoint_out", 10, [this](PoseStamped::ConstSharedPtr m) { out.push_back(*m); });
    decision_sub = harness->create_subscription<std_msgs::msg::String>(
        "geofence_guard/decisions", 10,
        [this](std_msgs::msg::String::ConstSharedPtr m) { decisions.push_back(m->data); });

    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(guard);
    executor->add_node(harness);
    // Let discovery settle so the first message is not lost.
    spin_until([this] {
      return odom_pub->get_subscription_count() > 0 && sp_pub->get_subscription_count() > 0 &&
             out_sub->get_publisher_count() > 0 && decision_sub->get_publisher_count() > 0;
    });
  }

  bool spin_until(const std::function<bool()>& done, std::chrono::milliseconds limit = 3000ms) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->spin_some(10ms);
      if (done()) return true;
    }
    return false;
  }

  void spin_for(std::chrono::milliseconds d) { spin_until([] { return false; }, d); }

  void send_odometry(double x, double y, double z, const std::string& frame = "map") {
    Odometry o;
    o.header.frame_id = frame;
    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = z;
    o.pose.pose.orientation.w = 1.0;
    odom_pub->publish(o);
    spin_for(50ms);
  }

  // Publishes one setpoint and waits for the decision on it.
  std::string send_setpoint(double x, double y, double z, const std::string& frame = "map") {
    PoseStamped p;
    p.header.frame_id = frame;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    p.pose.orientation.w = 1.0;
    const auto before = decisions.size();
    sp_pub->publish(p);
    EXPECT_TRUE(spin_until([&] { return decisions.size() > before; })) << "no decision published";
    spin_for(30ms);  // the forwarded setpoint, if any, is a separate message
    return decisions.empty() ? "" : decisions.back();
  }

  static bool contains(const std::string& s, const std::string& needle) {
    return s.find(needle) != std::string::npos;
  }

  std::shared_ptr<geofence_guard_ros::GuardNode> guard;
  std::shared_ptr<rclcpp::Node> harness;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<PoseStamped>::SharedPtr sp_pub;
  rclcpp::Subscription<PoseStamped>::SharedPtr out_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::vector<PoseStamped> out;
  std::vector<std::string> decisions;
};

}  // namespace

TEST_F(GuardNodeTest, ForwardsAnAcceptedSetpointUnchanged) {
  send_odometry(20, 20, 30);
  const auto d = send_setpoint(20, 200, 30);
  EXPECT_TRUE(contains(d, "\"accepted\":true")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"forward\"")) << d;
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].pose.position.y, 200.0);
  EXPECT_EQ(out[0].header.frame_id, "map");
}

TEST_F(GuardNodeTest, RefusedShortcutHoldsAtTheVehiclePosition) {
  send_odometry(20, 230, 30);
  const auto d = send_setpoint(280, 100, 30);
  EXPECT_TRUE(contains(d, "\"accepted\":false")) << d;
  EXPECT_TRUE(contains(d, "path_leaves_fence")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"hold\"")) << d;
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].pose.position.x, 20.0);
  EXPECT_DOUBLE_EQ(out[0].pose.position.y, 230.0);
  EXPECT_DOUBLE_EQ(out[0].pose.position.z, 30.0);
}

TEST_F(GuardNodeTest, HoldPointIsLatchedUntilASetpointIsAccepted) {
  send_odometry(20, 230, 30);
  send_setpoint(280, 100, 30);
  // The vehicle keeps drifting while refused setpoints keep arriving.
  send_odometry(24, 226, 30);
  send_setpoint(280, 100, 30);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out[1].pose.position.x, 20.0) << "hold point moved with the drift";

  send_setpoint(30, 200, 30);  // accepted: releases the latch
  send_odometry(30, 200, 30);
  send_setpoint(280, 100, 30);
  ASSERT_EQ(out.size(), 4u);
  EXPECT_DOUBLE_EQ(out[3].pose.position.x, 30.0);
  EXPECT_DOUBLE_EQ(out[3].pose.position.y, 200.0);
}

TEST_F(GuardNodeTest, DropsEverythingBeforeTheFirstOdometry) {
  const auto d = send_setpoint(20, 200, 30);
  EXPECT_TRUE(contains(d, "no vehicle state received yet")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"drop\"")) << d;
  EXPECT_TRUE(out.empty());
}

TEST_F(GuardNodeTest, StaleOdometryDropsRatherThanHolds) {
  // field.fence allows 0.5 s. A hold computed from an old position could be
  // anywhere, so the guard goes quiet and lets the autopilot's failsafe act.
  send_odometry(20, 20, 30);
  spin_for(700ms);
  const auto d = send_setpoint(20, 200, 30);
  EXPECT_TRUE(contains(d, "stale_state")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"drop\"")) << d;
  EXPECT_TRUE(out.empty());
}

TEST_F(GuardNodeTest, SetpointInTheWrongFrameIsRefused) {
  send_odometry(20, 20, 30);
  const auto d = send_setpoint(5, 0, 0, "base_link");
  EXPECT_TRUE(contains(d, "frame_mismatch")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"hold\"")) << d;
}

TEST_F(GuardNodeTest, OdometryInTheWrongFrameIsNeverHeldOn) {
  send_odometry(20, 20, 30, "odom");
  const auto d = send_setpoint(20, 200, 30);
  EXPECT_TRUE(contains(d, "frame_mismatch")) << d;
  EXPECT_TRUE(contains(d, "\"action\":\"drop\"")) << d;
  EXPECT_TRUE(out.empty());
}

TEST_F(GuardNodeTest, NonFiniteSetpointIsRefusedAndReportedAsNull) {
  send_odometry(20, 20, 30);
  const auto d = send_setpoint(std::nan(""), 200, 30);
  EXPECT_TRUE(contains(d, "non_finite")) << d;
  EXPECT_TRUE(contains(d, "\"target\":[null,")) << d;
}

TEST_F(GuardNodeTest, HoldCanBeTurnedOff) {
  guard.reset();
  executor.reset();
  out.clear();
  decisions.clear();
  start({{"hold_on_refusal", false}});
  send_odometry(20, 230, 30);
  const auto d = send_setpoint(280, 100, 30);
  EXPECT_TRUE(contains(d, "\"action\":\"drop\"")) << d;
  EXPECT_TRUE(out.empty());
}

TEST(GuardNodeStartup, RefusesToStartWithoutAnEnvelope) {
  rclcpp::init(0, nullptr);
  EXPECT_THROW(geofence_guard_ros::GuardNode{}, std::runtime_error);
  EXPECT_THROW(geofence_guard_ros::GuardNode(rclcpp::NodeOptions().parameter_overrides(
                   {{"envelope_file", std::string("/nonexistent.fence")}})),
               gfg::EnvelopeError);
  rclcpp::shutdown();
}
