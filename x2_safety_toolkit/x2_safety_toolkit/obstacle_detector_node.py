#!/usr/bin/env python3
"""
Obstacle detector for the chest-front LiDAR.

Subscribes to the raw point cloud, filters a frontal detection zone,
and publishes distances only — it never commands the robot.

Publishes:
  /x2/obstacle_status  (std_msgs/String, JSON)
      {"stamp", "frame_id", "min_front_distance",
       "left_clearance", "right_clearance", "state"}
  /x2/debug_points      (sensor_msgs/PointCloud2)
      the subset of points that fell inside the detection zone, for rviz.

Run:
  ros2 run x2_safety_toolkit obstacle_detector_node
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from x2_safety_toolkit.common import (
    DEBUG_POINTS_TOPIC,
    FRONT_SLOW_DISTANCE,
    FRONT_STOP_DISTANCE,
    LIDAR_TOPIC,
    OBSTACLE_STATUS_TOPIC,
    ZONE_X_MAX,
    ZONE_X_MIN,
    ZONE_Y_MAX,
    ZONE_Y_MIN,
    ZONE_Z_MAX,
    ZONE_Z_MIN,
    build_obstacle_status_json,
    classify_distance,
    in_zone,
    sensor_qos,
)


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")

        self.declare_parameter("front_stop_distance", FRONT_STOP_DISTANCE)
        self.declare_parameter("front_slow_distance", FRONT_SLOW_DISTANCE)
        self.declare_parameter("zone_x_min", ZONE_X_MIN)
        self.declare_parameter("zone_x_max", ZONE_X_MAX)
        self.declare_parameter("zone_y_min", ZONE_Y_MIN)
        self.declare_parameter("zone_y_max", ZONE_Y_MAX)
        self.declare_parameter("zone_z_min", ZONE_Z_MIN)
        self.declare_parameter("zone_z_max", ZONE_Z_MAX)

        self.status_pub = self.create_publisher(
            String, OBSTACLE_STATUS_TOPIC, 10)
        self.debug_points_pub = self.create_publisher(
            PointCloud2, DEBUG_POINTS_TOPIC, 10)

        self.sub = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.cb_pointcloud, sensor_qos())

        self.get_logger().info(
            f"Obstacle detector started (perception only, no motion commands). "
            f"Subscribed to {LIDAR_TOPIC}, publishing {OBSTACLE_STATUS_TOPIC} "
            f"and {DEBUG_POINTS_TOPIC}."
        )

    def zone_bounds(self):
        gp = self.get_parameter
        return (
            gp("zone_x_min").value, gp("zone_x_max").value,
            gp("zone_y_min").value, gp("zone_y_max").value,
            gp("zone_z_min").value, gp("zone_z_max").value,
        )

    def cb_pointcloud(self, msg: PointCloud2):
        x_min, x_max, y_min, y_max, z_min, z_max = self.zone_bounds()

        min_front = None
        min_left = None
        min_right = None
        zone_points = []

        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if not in_zone(x, y, z, x_min, x_max, y_min, y_max, z_min, z_max):
                continue

            zone_points.append((x, y, z))
            dist = math.sqrt(x * x + y * y)

            if min_front is None or dist < min_front:
                min_front = dist
            if y > 0.0 and (min_left is None or dist < min_left):
                min_left = dist
            if y < 0.0 and (min_right is None or dist < min_right):
                min_right = dist

        stop_distance = self.get_parameter("front_stop_distance").value
        slow_distance = self.get_parameter("front_slow_distance").value
        state = classify_distance(min_front, stop_distance, slow_distance)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        status = String()
        status.data = build_obstacle_status_json(
            stamp_sec, msg.header.frame_id, min_front, min_left, min_right, state)
        self.status_pub.publish(status)

        debug_msg = point_cloud2.create_cloud_xyz32(msg.header, zone_points)
        self.debug_points_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
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
