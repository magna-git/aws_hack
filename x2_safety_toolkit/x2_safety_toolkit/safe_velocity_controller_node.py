#!/usr/bin/env python3
"""
Safe velocity controller — the ONLY node in x2_safety_toolkit allowed to
move the robot, and only when explicitly started with dry_run:=false.

Subscribes to /x2/obstacle_status (published by obstacle_detector_node)
and decides:
  distance < front_stop_distance                      -> stop + turn
  front_stop_distance <= distance < front_slow_distance -> slow forward
  otherwise (or no obstacle data yet)                  -> forward

dry_run is True by default. In dry_run mode this node only *prints* the
command it would send — it never publishes to
/aima/mc/locomotion/velocity. Nothing here starts the robot moving on
its own; you must explicitly pass --ros-args -p dry_run:=false, and the
robot must already be in Stable Standing Mode.

Run (perception check, always safe):
  ros2 run x2_safety_toolkit safe_velocity_controller_node --ros-args -p dry_run:=true

Run (live, only after Stable Standing Mode):
  ros2 run py_examples set_mc_action SD
  ros2 run x2_safety_toolkit safe_velocity_controller_node --ros-args -p dry_run:=false
"""

import signal
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aimdk_msgs.msg import McInputAction, McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource

from x2_safety_toolkit.common import (
    FORWARD_SPEED,
    FRONT_SLOW_DISTANCE,
    FRONT_STOP_DISTANCE,
    OBSTACLE_STATUS_TOPIC,
    SLOW_SPEED,
    STATE_SLOW,
    STATE_STOP,
    TURN_SPEED,
    VELOCITY_TOPIC,
    classify_distance,
    parse_obstacle_status_json,
)

INPUT_SOURCE_SERVICE = "/aimdk_5Fmsgs/srv/SetMcInputSource"
INPUT_SOURCE_NAME = "x2_safety_toolkit"

STALE_DATA_TIMEOUT_SEC = 1.0


class SafeVelocityControllerNode(Node):
    def __init__(self):
        super().__init__("safe_velocity_controller_node")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("front_stop_distance", FRONT_STOP_DISTANCE)
        self.declare_parameter("front_slow_distance", FRONT_SLOW_DISTANCE)
        self.declare_parameter("forward_speed", FORWARD_SPEED)
        self.declare_parameter("slow_speed", SLOW_SPEED)
        self.declare_parameter("turn_speed", TURN_SPEED)

        self.dry_run = self.get_parameter("dry_run").value
        self.last_status_time = None
        self.velocity_pub = None
        self.input_source_client = None

        if self.dry_run:
            self.get_logger().warn(
                "DRY_RUN=true: no command will ever be published to "
                f"{VELOCITY_TOPIC}. This node only prints intended commands."
            )
        else:
            self.get_logger().warn(
                "DRY_RUN=false: this node WILL publish real velocity "
                f"commands to {VELOCITY_TOPIC}. Make sure the robot is in "
                "Stable Standing Mode and someone is ready to stop it."
            )
            self.velocity_pub = self.create_publisher(
                McLocomotionVelocity, VELOCITY_TOPIC, 10)
            self.input_source_client = self.create_client(
                SetMcInputSource, INPUT_SOURCE_SERVICE)

        self.sub = self.create_subscription(
            String, OBSTACLE_STATUS_TOPIC, self.cb_obstacle_status, 10)

        self.watchdog_timer = self.create_timer(0.5, self.check_stale_data)

        self.get_logger().info(
            f"Safe velocity controller started. Subscribed to "
            f"{OBSTACLE_STATUS_TOPIC}."
        )

        if not self.dry_run:
            self.register_input_source()

    def register_input_source(self, timeout_sec: float = 3.0) -> bool:
        """Best-effort registration with SetMcInputSource. Never blocks startup."""
        if not self.input_source_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Service {INPUT_SOURCE_SERVICE} not available after "
                f"{timeout_sec:.1f}s — continuing without input-source "
                "registration (velocity commands may be ignored by the MC)."
            )
            return False

        req = SetMcInputSource.Request()
        req.action = McInputAction()
        req.action.value = McInputAction.INPUTACTION_ADD
        req.input_source.name = INPUT_SOURCE_NAME
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        future = None
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.input_source_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Registering input source... retry [{i}]")

        if future is not None and future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Input source '{INPUT_SOURCE_NAME}' registered: "
                    f"task_id={response.response.task_id}"
                )
                return True
            except Exception as e:
                self.get_logger().warn(f"SetMcInputSource call raised: {e}")
                return False

        self.get_logger().warn(
            "SetMcInputSource call timed out — continuing without "
            "registration."
        )
        return False

    def cb_obstacle_status(self, msg: String):
        self.last_status_time = self.get_clock().now()

        try:
            status = parse_obstacle_status_json(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Malformed obstacle status JSON: {e}")
            return

        distance = status.get("min_front_distance")
        self.apply_command(distance)

    def apply_command(self, distance):
        stop_distance = self.get_parameter("front_stop_distance").value
        slow_distance = self.get_parameter("front_slow_distance").value
        forward_speed = self.get_parameter("forward_speed").value
        slow_speed = self.get_parameter("slow_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        state = classify_distance(distance, stop_distance, slow_distance)
        if state == STATE_STOP:
            forward, lateral, angular = 0.0, 0.0, turn_speed
        elif state == STATE_SLOW:
            forward, lateral, angular = slow_speed, 0.0, 0.0
        else:
            forward, lateral, angular = forward_speed, 0.0, 0.0

        dist_str = f"{distance:.2f}m" if distance is not None else "n/a"
        if self.dry_run:
            self.get_logger().info(
                f"[DRY_RUN] state={state} distance={dist_str} -> would publish "
                f"forward={forward:.2f} lateral={lateral:.2f} "
                f"angular={angular:.2f} on {VELOCITY_TOPIC}"
            )
        else:
            self.publish_velocity(forward, lateral, angular)
            self.get_logger().info(
                f"[LIVE] state={state} distance={dist_str} -> published "
                f"forward={forward:.2f} lateral={lateral:.2f} angular={angular:.2f}"
            )

    def check_stale_data(self):
        if self.dry_run or self.last_status_time is None:
            return
        age_sec = (self.get_clock().now() - self.last_status_time).nanoseconds * 1e-9
        if age_sec > STALE_DATA_TIMEOUT_SEC:
            self.get_logger().warn(
                f"{OBSTACLE_STATUS_TOPIC} is stale ({age_sec:.2f}s old) — "
                "publishing stop for safety."
            )
            self.publish_stop()

    def publish_velocity(self, forward: float, lateral: float, angular: float):
        if self.dry_run or self.velocity_pub is None:
            return
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = INPUT_SOURCE_NAME
        msg.forward_velocity = forward
        msg.lateral_velocity = lateral
        msg.angular_velocity = angular
        self.velocity_pub.publish(msg)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0, 0.0)


_global_node = None


def _signal_handler(sig, frame):
    global _global_node
    if _global_node is not None:
        try:
            if not _global_node.dry_run:
                _global_node.publish_stop()
                _global_node.get_logger().info(
                    f"Signal {sig} received — stop command published, "
                    "shutting down."
                )
            else:
                _global_node.get_logger().info(
                    f"Signal {sig} received — dry_run mode, nothing was "
                    "ever published, shutting down."
                )
        except Exception:
            pass
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    global _global_node
    rclpy.init(args=args)

    node = SafeVelocityControllerNode()
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not node.dry_run:
                node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
