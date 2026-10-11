#!/usr/bin/env bash
# Fly sitl/mission.csv in PX4 SITL, once straight to the autopilot and once
# through geofence-guard, logging telemetry for each run.
#
#   sitl/run.sh <out_dir> [unguarded|guarded|both]
#
# Needs Docker and the geofence-guard-sitl:dev image (make sitl-image).
# PX4 is restarted for each run so both start from the same state.
set -euo pipefail

out=$(mkdir -p "$1" && cd "$1" && pwd)
mode=${2:-both}
here=$(cd "$(dirname "$0")" && pwd)
px4_image=${PX4_IMAGE:-jonasvautherin/px4-gazebo-headless:1.15.4}
ros_image=${ROS_IMAGE:-geofence-guard-sitl:dev}
# Keep this run's DDS traffic away from anything else on the host network.
domain=${ROS_DOMAIN_ID:-73}

cleanup() { docker rm -f gfg-px4 >/dev/null 2>&1 || true; }
trap cleanup EXIT

fly() {
  local name=$1 guarded=$2
  echo "== $name"
  cleanup
  # -t: PX4's shell spins on a prompt without a terminal and floods the log.
  docker run -d -t --name gfg-px4 --network host "$px4_image" 127.0.0.1 127.0.0.1 >/dev/null
  for _ in $(seq 120); do
    docker logs gfg-px4 2>&1 | grep -q "Ready for takeoff" && break
    sleep 2
  done
  docker logs gfg-px4 2>&1 | grep -q "Ready for takeoff" || { echo "PX4 did not become ready"; exit 1; }

  docker run --rm --network host -e ROS_DOMAIN_ID="$domain" -e HOME=/tmp -u "$(id -u):$(id -g)" \
    -v "$here":/sitl:ro -v "$out":/out "$ros_image" bash -c "
      set -e
      source /ws/install/setup.bash
      ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580 \
        > /out/$name-mavros.log 2>&1 &
      if [ $guarded = 1 ]; then
        ros2 run geofence_guard_ros geofence_guard_node --ros-args \
          -p envelope_file:=/sitl/field.fence \
          -r odometry:=/mavros/local_position/odom \
          -r setpoint_out:=/mavros/setpoint_position/local \
          > /out/$name-guard.log 2>&1 &
        flag=--guarded
      fi
      python3 /sitl/planner.py --mission /sitl/mission.csv \
        --log /out/$name.csv --events /out/$name-events.json \${flag:-}
    "
  docker logs gfg-px4 > "$out/$name-px4.log" 2>&1 || true
}

case $mode in
  unguarded) fly unguarded 0 ;;
  guarded) fly guarded 1 ;;
  both) fly unguarded 0; fly guarded 1 ;;
  *) echo "unknown mode $mode"; exit 2 ;;
esac
