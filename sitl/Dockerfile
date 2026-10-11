# ROS 2 Jazzy + MAVROS, with the geofence-guard workspace built in.
# The SITL itself runs in a separate container (see run.sh).
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-mavros ros-jazzy-mavros-extras python3-matplotlib python3-pil ffmpeg xdotool fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/* \
    && /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /ws
COPY core src/geofence-guard/core
COPY ros2 src/geofence-guard/ros2
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DGFG_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log
