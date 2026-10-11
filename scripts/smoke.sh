#!/usr/bin/env bash
# End-to-end check of the installed package as a separate process: launch the
# node, drive it with the ros2 CLI, and look at what it publishes. Run from a
# sourced workspace (CI runs it in the ros:jazzy container).
set -euo pipefail

ros2 launch geofence_guard_ros geofence_guard.launch.py > /tmp/guard.log 2>&1 &
guard=$!
trap 'kill $guard 2>/dev/null || true' EXIT

for _ in $(seq 50); do
  ros2 node list 2>/dev/null | grep -q /geofence_guard && break
  sleep 0.2
done
ros2 node list | grep -q /geofence_guard || { cat /tmp/guard.log; echo "node did not start"; exit 1; }

odom='{header: {frame_id: map}, pose: {pose: {position: {x: 20.0, y: 230.0, z: 30.0}, orientation: {w: 1.0}}}}'
ros2 topic pub -r 20 /odometry nav_msgs/msg/Odometry "$odom" > /dev/null &
odom_pid=$!
trap 'kill $guard $odom_pid 2>/dev/null || true' EXIT
sleep 1

expect() {  # expect <name> <setpoint yaml> <pattern in decision>
  local got
  ros2 topic pub -r 10 /setpoint_in geometry_msgs/msg/PoseStamped "$2" > /dev/null &
  local pub=$!
  got=$(timeout 10 ros2 topic echo --once --field data /geofence_guard/decisions std_msgs/msg/String)
  kill $pub
  wait $pub 2>/dev/null || true
  if grep -q "$3" <<< "$got"; then
    echo "ok    $1"
  else
    echo "FAIL  $1: wanted '$3', got: $got"
    exit 1
  fi
}

expect "accepted leg is forwarded" \
  '{header: {frame_id: map}, pose: {position: {x: 20.0, y: 100.0, z: 30.0}}}' '"action":"forward"'
expect "shortcut across the notch holds" \
  '{header: {frame_id: map}, pose: {position: {x: 280.0, y: 100.0, z: 30.0}}}' '"action":"hold".*path_leaves_fence'
expect "setpoint in body frame is refused" \
  '{header: {frame_id: base_link}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}' 'frame_mismatch'

kill $odom_pid
sleep 1
expect "stale odometry drops" \
  '{header: {frame_id: map}, pose: {position: {x: 20.0, y: 100.0, z: 30.0}}}' '"action":"drop"'

echo "smoke: all ok"
