#!/usr/bin/env python3
"""
Read-only inspector for the chest-front LiDAR.

Checks that /aima/hal/sensor/lidar_chest_front/lidar_pointcloud exists and
is actually publishing, then reports message type, frequency, frame_id and
an approximate point count. Never publishes any command.

Run:
  ros2 run x2_safety_toolkit lidar_inspector_node
"""

from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from x2_safety_toolkit.common import LIDAR_TOPIC, sensor_qos

REPORT_PERIOD_SEC = 1.0
NO_DATA_WARN_SEC = 5.0


class LidarInspectorNode(Node):
    def __init__(self):
        super().__init__("lidar_inspector_node")

        self.arrivals = deque()
        self.last_report_time = self.get_clock().now()
        self.last_msg_time = None

        self.check_topic_exists()

        self.sub = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.cb_pointcloud, sensor_qos())

        self.no_data_timer = self.create_timer(
            NO_DATA_WARN_SEC, self.check_no_data)

        self.get_logger().info(
            f"Inspecting {LIDAR_TOPIC} (read-only, publishes nothing)."
        )

    def check_topic_exists(self):
        topics = dict(self.get_topic_names_and_types())
        if LIDAR_TOPIC not in topics:
            self.get_logger().warn(
                f"{LIDAR_TOPIC} is not advertised yet by any publisher. "
                "Waiting — it may appear once the sensor driver starts."
            )
        else:
            self.get_logger().info(
                f"Topic found: {LIDAR_TOPIC} types={topics[LIDAR_TOPIC]}"
            )

    def check_no_data(self):
        if self.last_msg_time is None:
            self.get_logger().warn(
                f"No data received on {LIDAR_TOPIC} yet — "
                "check 'ros2 topic hz {topic}' and the sensor driver."
            )

    def update_fps(self, now):
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()
        return len(self.arrivals)

    def cb_pointcloud(self, msg: PointCloud2):
        now = self.get_clock().now()
        self.last_msg_time = now
        fps = self.update_fps(now)

        if (now - self.last_report_time).nanoseconds * 1e-9 < REPORT_PERIOD_SEC:
            return
        self.last_report_time = now

        fields_str = ", ".join(f.name for f in msg.fields)
        approx_points = msg.width * msg.height

        self.get_logger().info(
            f"type=sensor_msgs/PointCloud2 frame_id='{msg.header.frame_id}' "
            f"fps~={fps:.1f}Hz approx_points={approx_points} "
            f"(width={msg.width} height={msg.height}) fields=[{fields_str}] "
            f"is_dense={msg.is_dense}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
