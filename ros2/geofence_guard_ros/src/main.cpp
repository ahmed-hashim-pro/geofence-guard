#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geofence_guard_ros/guard_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<geofence_guard_ros::GuardNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("geofence_guard"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
