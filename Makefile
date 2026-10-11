BUILD ?= build
ROS_IMAGE ?= ros:jazzy-ros-base

.PHONY: build test check ros demo sitl-image sitl sitl-video clean

build:
	cmake -S core -B $(BUILD) -DCMAKE_BUILD_TYPE=Release -DGFG_WERROR=ON
	cmake --build $(BUILD)

test: build
	ctest --test-dir $(BUILD) --output-on-failure

# The core suite under AddressSanitizer and UBSan.
check:
	cmake -S core -B $(BUILD)-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo -DGFG_WERROR=ON -DGFG_SANITIZE=ON
	cmake --build $(BUILD)-asan
	ctest --test-dir $(BUILD)-asan --output-on-failure

# Build, test and launch the ROS 2 node in a Jazzy container. Needs Docker, not ROS.
ros:
	docker run --rm -v $(CURDIR):/ws/src/geofence-guard:ro -w /ws $(ROS_IMAGE) bash -c '\
	  source /opt/ros/jazzy/setup.bash && \
	  colcon build --cmake-args -DGFG_BUILD_TESTS=OFF && \
	  colcon test --packages-select geofence_guard_ros && colcon test-result --verbose && \
	  source install/setup.bash && src/geofence-guard/scripts/smoke.sh'

# Re-record docs/demo.gif and re-render the README diagrams from the real CLI.
# Needs Docker for VHS; the CLI is linked statically so it runs in that image.
demo: build
	$(BUILD)/geofence-guard render examples/field.fence examples/shortcut.csv > docs/shortcut.svg
	$(BUILD)/geofence-guard render examples/field.fence examples/survey.csv > docs/survey.svg
	mkdir -p $(BUILD)/static
	$(CXX) -std=c++17 -O2 -static -Icore/include -I$(BUILD)/generated core/src/*.cpp core/cli/*.cpp \
	  -o $(BUILD)/static/geofence-guard
	docker run --rm -u $$(id -u):$$(id -g) -e HOME=/tmp -v $(CURDIR):/vhs \
	  -v $(CURDIR)/$(BUILD)/static:/opt/gfg:ro ghcr.io/charmbracelet/vhs:v0.10.0 docs/demo.tape

# PX4 SITL: fly sitl/mission.csv without and with the guard, check the guarded
# run never left the fence, and re-render docs/sitl.gif. Needs Docker only;
# the PX4 image is about 5 GB.
SITL_OUT ?= $(BUILD)/sitl
sitl-image:
	docker build -f sitl/Dockerfile -t geofence-guard-sitl:dev .

sitl: sitl-image
	sitl/run.sh $(SITL_OUT) both
	docker run --rm -u $$(id -u):$$(id -g) -e HOME=/tmp -e MPLCONFIGDIR=/tmp -v $(CURDIR):/repo \
	  -v $(CURDIR)/$(SITL_OUT):/out geofence-guard-sitl:dev \
	  python3 /repo/sitl/analyze.py /out --check --gif /repo/docs/sitl.gif
	cp $(SITL_OUT)/results.json docs/sitl-results.json

# Record Gazebo's 3D view of both runs and compose docs/sitl-gazebo.{mp4,gif}.
sitl-video: sitl-image
	RECORD=1 sitl/run.sh $(SITL_OUT)-video both
	docker run --rm -u $$(id -u):$$(id -g) -e HOME=/tmp -e MPLCONFIGDIR=/tmp -v $(CURDIR):/repo \
	  -v $(CURDIR)/$(SITL_OUT)-video:/out -w /repo/sitl geofence-guard-sitl:dev bash -c '\
	  python3 analyze.py /out --check && \
	  python3 render_video.py /out /repo/docs/sitl-gazebo.mp4 --gif /repo/docs/sitl-gazebo.gif'

clean:
	rm -rf $(BUILD) $(BUILD)-asan
