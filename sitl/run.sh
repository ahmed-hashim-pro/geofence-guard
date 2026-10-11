#!/usr/bin/env bash
# Fly sitl/mission.csv in PX4 SITL, once straight to the autopilot and once
# through geofence-guard, logging telemetry for each run.
#
#   sitl/run.sh <out_dir> [unguarded|guarded|both]
#
# Needs Docker and the geofence-guard-sitl:dev image (make sitl-image).
# PX4 is restarted for each run so both start from the same state.
#
# RECORD=1 also loads sitl/fence.world (the envelope drawn as walls, see
# make_world.py), opens Gazebo's GUI on the container's virtual display and
# records it to <out_dir>/<run>.mp4 with ffmpeg; render_video.py turns the
# two recordings into the side-by-side clip.
set -euo pipefail

out=$(mkdir -p "$1" && cd "$1" && pwd)
mode=${2:-both}
here=$(cd "$(dirname "$0")" && pwd)
px4_image=${PX4_IMAGE:-jonasvautherin/px4-gazebo-headless:1.15.4}
ros_image=${ROS_IMAGE:-geofence-guard-sitl:dev}
# Keep this run's DDS traffic away from anything else on the host network.
domain=${ROS_DOMAIN_ID:-73}

record=${RECORD:-0}
world=/root/Firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world
# The X socket directory is shared with the PX4 container, whose Xvfb runs as
# root and takes it over, so each invocation gets a fresh one.
x11=$(mktemp -d)
chmod 1777 "$x11"
px4_args=()
if [ "$record" = 1 ]; then
  # The world is generated, not committed; rebuild it from the envelope.
  python3 "$here/make_world.py" "$here/field.fence" "$out/fence.world"
  # At the camera's range the 0.5 m airframe is a few pixels, so the recorded
  # vehicle carries a 10 m marker sphere: a visual on base_link, with no
  # collision and no mass, so it changes nothing about how the vehicle flies.
  iris=/root/Firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
  docker run --rm --entrypoint cat "$px4_image" "$iris" | python3 -c '
import sys
sdf = sys.stdin.read()
marker = """<visual name="video_marker">
        <cast_shadows>false</cast_shadows>
        <geometry><sphere><radius>5.0</radius></sphere></geometry>
        <material><ambient>0.1 0.35 0.95 1</ambient><diffuse>0.1 0.35 0.95 1</diffuse>
          <emissive>0.05 0.2 0.6 1</emissive></material>
      </visual>
      <collision name="base_link_inertia_collision">"""
anchor = "<collision name=\x27base_link_inertia_collision\x27>"
assert anchor in sdf, "iris.sdf changed: no base_link collision anchor"
sys.stdout.write(sdf.replace(anchor, marker, 1))' > "$out/iris.sdf"
  px4_args=(-v "$out/fence.world:$world:ro" -v "$out/iris.sdf:$iris:ro" -v "$x11:/tmp/.X11-unix")
fi

cleanup() { docker rm -f gfg-px4 >/dev/null 2>&1 || true; }
trap cleanup EXIT

fly() {
  local name=$1 guarded=$2
  echo "== $name"
  cleanup
  # -t: PX4's shell spins on a prompt without a terminal and floods the log.
  docker run -d -t --name gfg-px4 --network host "${px4_args[@]}" "$px4_image" 127.0.0.1 127.0.0.1 >/dev/null
  for _ in $(seq 120); do
    docker logs gfg-px4 2>&1 | grep -q "Ready for takeoff" && break
    sleep 2
  done
  docker logs gfg-px4 2>&1 | grep -q "Ready for takeoff" || { echo "PX4 did not become ready"; exit 1; }

  if [ "$record" = 1 ]; then
    # Gazebo's GUI on the Xvfb display the image already runs, sized so the
    # 3D view is a known rectangle that ffmpeg can crop to.
    docker exec -d gfg-px4 bash -c '
      source /root/Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash \
        /root/Firmware /root/Firmware/build/px4_sitl_default > /dev/null
      mkdir -p /root/.gazebo
      printf "[geometry]\nx=0\ny=0\nwidth=1600\nheight=1200\n" > /root/.gazebo/gui.ini
      DISPLAY=:99 gzclient > /tmp/gzclient.log 2>&1'
    sleep 15
  fi

  docker run --rm --network host -e ROS_DOMAIN_ID="$domain" -e HOME=/tmp -u "$(id -u):$(id -g)" \
    -e DISPLAY=:99 -v "$x11":/tmp/.X11-unix \
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
      if [ $record = 1 ]; then
        # The 3D view of a 1600x1200 gzclient window, without menus and panels.
        # Xvfb paints its cursor into the framebuffer; park it outside the crop.
        xdotool mousemove 1599 1199 || true
        date +%s.%N > /out/$name-video-start
        ffmpeg -hide_banner -loglevel error -y -f x11grab -draw_mouse 0 -framerate 20 \
          -video_size 1310x1050 -i :99+270,73 -c:v libx264 -preset veryfast -crf 23 \
          -pix_fmt yuv420p /out/$name.mp4 < /dev/null > /out/$name-ffmpeg.log 2>&1 &
        rec=\$!
      fi
      python3 /sitl/planner.py --mission /sitl/mission.csv \
        --log /out/$name.csv --events /out/$name-events.json \${flag:-}
      if [ $record = 1 ]; then
        sleep 2
        kill -INT \$rec; wait \$rec || true
      fi
    "
  docker logs gfg-px4 > "$out/$name-px4.log" 2>&1 || true
}

case $mode in
  unguarded) fly unguarded 0 ;;
  guarded) fly guarded 1 ;;
  both) fly unguarded 0; fly guarded 1 ;;
  *) echo "unknown mode $mode"; exit 2 ;;
esac
