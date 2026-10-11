#!/usr/bin/env python3
"""A deliberately naive mission planner for the SITL runs.

It streams position setpoints for one waypoint at a time, switches PX4 to
OFFBOARD and arms, and moves on when the vehicle is within 2 m of the target.
It knows nothing about the fence. When it runs behind the guard, it also
listens to the guard's decisions and gives up on a waypoint that has been
refused for 3 s, which is what a planner that wants to keep flying would do.

Every odometry sample is logged with the waypoint being flown and the guard's
latest action, so the run can be analysed and replayed afterwards.
"""

import argparse
import csv
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

ARRIVED_M = 2.0
# The last waypoint also has to be held nearly still before landing: switching
# to AUTO.LAND at speed hands an overshoot to the autopilot, outside the guard.
SETTLED_MPS = 0.5
GIVE_UP_REFUSED_S = 3.0
LEG_TIMEOUT_S = 120.0
LAND_TIMEOUT_S = 60.0


def load_mission(path):
    points = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                points.append(tuple(float(v) for v in line.split(",")))
    return points


class Planner(Node):
    def __init__(self, mission, setpoint_topic, guarded, log_path):
        super().__init__("planner")
        self.mission = mission
        self.guarded = guarded
        self.index = 0
        self.leg_started = None
        self.refused_since = None
        self.last_action = "none" if guarded else "unguarded"
        self.last_codes = ""
        self.position = None
        self.speed = 0.0
        self.state = State()
        self.events = []
        self.t0 = time.monotonic()

        self.pub = self.create_publisher(PoseStamped, setpoint_topic, 10)
        self.create_subscription(
            Odometry, "/mavros/local_position/odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        if guarded:
            self.create_subscription(
                String, "/geofence_guard/decisions", self.on_decision, 10
            )
        self.arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode = self.create_client(SetMode, "/mavros/set_mode")

        self.log_file = open(log_path, "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(
            ["t", "wall", "x", "y", "z", "wp", "tx", "ty", "tz", "action", "codes", "mode"]
        )
        self.create_timer(0.05, self.tick)

    def now(self):
        return time.monotonic() - self.t0

    def event(self, text):
        self.get_logger().info(text)
        self.events.append({"t": round(self.now(), 2), "wall": round(time.time(), 3), "event": text})

    def target(self):
        return self.mission[min(self.index, len(self.mission) - 1)]

    def on_state(self, msg):
        self.state = msg

    def on_decision(self, msg):
        d = json.loads(msg.data)
        self.last_action = d["action"]
        self.last_codes = "|".join(v["code"] for v in d["violations"])

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        tx, ty, tz = self.target()
        self.log.writerow(
            [f"{self.now():.3f}", f"{time.time():.3f}", f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}", self.index,
             tx, ty, tz, self.last_action, self.last_codes, self.state.mode]
        )

    def tick(self):
        if self.index < len(self.mission):
            tx, ty, tz = self.target()
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = "map"
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = tx, ty, tz
            sp.pose.orientation.w = 1.0
            self.pub.publish(sp)

        if self.position is None or self.index >= len(self.mission):
            return
        if self.leg_started is None:
            self.leg_started = self.now()
            self.event(f"waypoint {self.index + 1}: {self.target()}")

        dist = math.dist(self.position, self.target())
        last = self.index == len(self.mission) - 1
        if dist < ARRIVED_M and (not last or self.speed < SETTLED_MPS):
            self.event(f"waypoint {self.index + 1} reached")
            self.advance()
            return

        if self.guarded and self.last_action in ("hold", "drop"):
            if self.refused_since is None:
                self.refused_since = self.now()
            elif self.now() - self.refused_since > GIVE_UP_REFUSED_S:
                self.event(f"waypoint {self.index + 1} refused by guard ({self.last_codes}); skipping")
                self.advance()
                return
        else:
            self.refused_since = None

        if self.now() - self.leg_started > LEG_TIMEOUT_S:
            self.event(f"waypoint {self.index + 1} timed out")
            self.advance()

    def advance(self):
        self.index += 1
        self.leg_started = None
        self.refused_since = None
        # The decision for the old target must not count against the new one.
        if self.guarded:
            self.last_action = "none"

    def call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{what}: service not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.event("waiting for FCU and odometry")
        while not (self.state.connected and self.position is not None):
            rclpy.spin_once(self, timeout_sec=0.1)
        # PX4 refuses OFFBOARD until setpoints have been streaming for a while.
        self.spin_for(2.0)

        for _ in range(20):
            if self.state.mode != "OFFBOARD":
                self.call(self.mode, SetMode.Request(custom_mode="OFFBOARD"), "set_mode")
            if not self.state.armed:
                self.call(self.arm, CommandBool.Request(value=True), "arming")
            self.spin_for(0.5)
            if self.state.armed and self.state.mode == "OFFBOARD":
                break
        else:
            raise RuntimeError(f"could not arm in OFFBOARD (mode {self.state.mode})")
        self.event("armed in OFFBOARD")

        while self.index < len(self.mission):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.event("mission done; landing")
        self.call(self.mode, SetMode.Request(custom_mode="AUTO.LAND"), "set_mode")
        end = time.monotonic() + LAND_TIMEOUT_S
        while self.state.armed and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.event("disarmed" if not self.state.armed else "land timed out")
        self.log_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", required=True)
    parser.add_argument("--log", required=True, help="telemetry CSV to write")
    parser.add_argument("--events", required=True, help="events JSON to write")
    parser.add_argument("--guarded", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    topic = "/setpoint_in" if args.guarded else "/mavros/setpoint_position/local"
    node = Planner(load_mission(args.mission), topic, args.guarded, args.log)
    code = 0
    try:
        node.run()
    except Exception as e:  # noqa: BLE001 - report and fail the run
        node.get_logger().error(str(e))
        code = 1
    with open(args.events, "w") as f:
        json.dump(node.events, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
