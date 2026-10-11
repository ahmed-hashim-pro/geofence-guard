// The ROS 2 side: sits between whatever produces position setpoints (a
// mission planner, an agent, a teleop bridge) and the autopilot's offboard
// input. Every setpoint is checked against the envelope before it is
// forwarded; nothing reaches setpoint_out without passing the Guard.
#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "geofence_guard/guard.hpp"

namespace geofence_guard_ros {

class GuardNode : public rclcpp::Node {
 public:
  // Throws if envelope_file is unset or the envelope is invalid: a guard that
  // starts without limits is worse than no guard, because it looks like one.
  explicit GuardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct Known {
    gfg::VehicleState state;
    geometry_msgs::msg::Quaternion orientation;
  };

  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void on_setpoint(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void publish_decision(const gfg::Decision& d, const char* action,
                        const geometry_msgs::msg::Point& target);

  gfg::Guard guard_;
  bool hold_on_refusal_;

  std::optional<Known> vehicle_;
  // Latched at the first refusal and kept until a setpoint is accepted, so a
  // stream of refused setpoints holds one point rather than chasing the drift.
  std::optional<geometry_msgs::msg::PoseStamped> hold_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
};

}  // namespace geofence_guard_ros
