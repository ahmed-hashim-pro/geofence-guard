"""Start the guard between a planner and an autopilot.

Remap the four topics to your stack, e.g. for a MAVROS setup:

  ros2 launch geofence_guard_ros geofence_guard.launch.py \
      envelope_file:=/path/to/field.fence \
      odometry:=/mavros/local_position/odom \
      setpoint_in:=/planner/setpoint \
      setpoint_out:=/mavros/setpoint_position/local
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = {
        "envelope_file": PathJoinSubstitution(
            [FindPackageShare("geofence_guard_ros"), "config", "field.fence"]
        ),
        "odometry": "odometry",
        "setpoint_in": "setpoint_in",
        "setpoint_out": "setpoint_out",
        "hold_on_refusal": "true",
    }
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in args.items()]

    guard = Node(
        package="geofence_guard_ros",
        executable="geofence_guard_node",
        name="geofence_guard",
        output="screen",
        parameters=[
            {
                "envelope_file": LaunchConfiguration("envelope_file"),
                "hold_on_refusal": LaunchConfiguration("hold_on_refusal"),
            }
        ],
        remappings=[
            ("odometry", LaunchConfiguration("odometry")),
            ("setpoint_in", LaunchConfiguration("setpoint_in")),
            ("setpoint_out", LaunchConfiguration("setpoint_out")),
        ],
    )
    return LaunchDescription(declared + [guard])
