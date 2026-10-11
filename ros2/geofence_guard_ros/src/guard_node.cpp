#include "geofence_guard_ros/guard_node.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace geofence_guard_ros {

namespace {

gfg::Envelope envelope_from_parameters(rclcpp::Node& node) {
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  read_only.description = "Path to the envelope file. Fixed for the life of the node.";
  const auto path = node.declare_parameter<std::string>("envelope_file", "", read_only);
  if (path.empty()) {
    throw std::runtime_error("parameter 'envelope_file' is required; refusing to start without limits");
  }
  return gfg::load_envelope(path);
}

std::string json_escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

std::string json_number(double v) {
  if (!std::isfinite(v)) return "null";
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.3f", v);
  return buf;
}

}  // namespace

GuardNode::GuardNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("geofence_guard", options),
      guard_(envelope_from_parameters(*this)),
      hold_on_refusal_(declare_parameter<bool>("hold_on_refusal", true)) {
  // Best-effort subscriptions accept both reliable and best-effort publishers,
  // which is what autopilot bridges tend to use for high-rate topics.
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { on_odometry(msg); });
  setpoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "setpoint_in", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { on_setpoint(msg); });
  setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("setpoint_out", 10);
  decision_pub_ = create_publisher<std_msgs::msg::String>("~/decisions", 10);

  const auto& env = guard_.envelope();
  RCLCPP_INFO(get_logger(),
              "envelope loaded: frame '%s', margin %.1f m, altitude %.1f-%.1f m, %zu fence "
              "vertices, %zu exclusion zones",
              env.frame.c_str(), env.margin, env.min_altitude, env.max_altitude, env.fence.size(),
              env.exclusions.size());
}

void GuardNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  // Freshness is measured from when the guard received the state, on the
  // guard's own clock. Header stamps from an autopilot bridge are often on a
  // different time base, and a guard that trusts them can be told any state
  // is fresh.
  Known k;
  k.state.frame = msg->header.frame_id;
  k.state.stamp = now().seconds();
  k.state.position = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z};
  k.orientation = msg->pose.pose.orientation;
  vehicle_ = k;
}

void GuardNode::on_setpoint(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  const double t = now().seconds();
  const auto& target = msg->pose.position;

  if (!vehicle_) {
    gfg::Decision d;
    d.violations.push_back({gfg::Reason::StaleState, "no vehicle state received yet"});
    publish_decision(d, "drop", target);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "dropping setpoints: no odometry received yet");
    return;
  }

  const gfg::Setpoint sp{msg->header.frame_id, t, {target.x, target.y, target.z}};
  const gfg::Decision d = guard_.check(vehicle_->state, sp);

  if (d.accepted) {
    hold_.reset();
    setpoint_pub_->publish(*msg);
    publish_decision(d, "forward", target);
    return;
  }

  // Holding only makes sense if the guard knows where the vehicle is, in the
  // envelope's frame, recently. Otherwise publish nothing: the autopilot's own
  // offboard-loss failsafe is the right authority when the guard is blind.
  const auto& s = vehicle_->state;
  const bool state_usable = s.frame == guard_.envelope().frame &&
                            std::isfinite(s.position.x) && std::isfinite(s.position.y) &&
                            std::isfinite(s.position.z) &&
                            t - s.stamp <= guard_.envelope().max_state_age;

  const char* action = "drop";
  if (hold_on_refusal_ && state_usable) {
    if (!hold_) {
      geometry_msgs::msg::PoseStamped h;
      h.header.frame_id = s.frame;
      h.pose.position.x = s.position.x;
      h.pose.position.y = s.position.y;
      h.pose.position.z = s.position.z;
      h.pose.orientation = vehicle_->orientation;
      hold_ = h;
    }
    hold_->header.stamp = now();
    setpoint_pub_->publish(*hold_);
    action = "hold";
  }
  publish_decision(d, action, target);

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "refused setpoint (%s): %s — %s",
                       gfg::reason_code(d.violations.front().reason),
                       d.violations.front().detail.c_str(), action);
}

void GuardNode::publish_decision(const gfg::Decision& d, const char* action,
                                 const geometry_msgs::msg::Point& target) {
  std::string json = "{\"accepted\":";
  json += d.accepted ? "true" : "false";
  json += ",\"recovery\":";
  json += d.recovery ? "true" : "false";
  json += ",\"action\":\"";
  json += action;
  json += "\",\"target\":[" + json_number(target.x) + "," + json_number(target.y) + "," +
          json_number(target.z) + "],\"violations\":[";
  for (std::size_t i = 0; i < d.violations.size(); ++i) {
    if (i > 0) json += ",";
    json += "{\"code\":\"";
    json += gfg::reason_code(d.violations[i].reason);
    json += "\",\"detail\":\"" + json_escape(d.violations[i].detail) + "\"}";
  }
  json += "]}";

  std_msgs::msg::String out;
  out.data = json;
  decision_pub_->publish(out);
}

}  // namespace geofence_guard_ros
