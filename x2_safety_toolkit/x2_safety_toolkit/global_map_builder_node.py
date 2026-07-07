#!/usr/bin/env python3
"""
Persistent global occupancy map ("fake SLAM"): mapping without the
official SLAM stack, using the robot's own leg odometry for pose instead
of computing localization ourselves.

Unlike local_safety_map_node (which redraws a small grid centered on the
robot at every scan), this node accumulates scans over time into one
large, fixed-world grid: each LiDAR point is transformed from the sensor
frame into the odometry frame using the latest /aima/mc/leg_odometry
pose, then rasterized into a persistent grid with simple ray-marked free
space.

Honest limitation: there is no scan-matching or loop closure here, so
the map will drift with the odometry's own drift over long distances.
For a bounded demo area this is a reasonable, fully local stand-in for
the (inaccessible) official SLAM/mapping stack.

Read-only / perception only: never commands the robot.

Publishes:
  /x2/global_map  (nav_msgs/OccupancyGrid)

Run:
  ros2 run x2_safety_toolkit global_map_builder_node
"""

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from x2_safety_toolkit.common import (
    GLOBAL_GRID_RESOLUTION,
    GLOBAL_GRID_SIZE,
    GLOBAL_MAP_TOPIC,
    GLOBAL_MAX_RANGE,
    GLOBAL_MIN_RANGE,
    LEG_ODOM_TOPIC,
    LIDAR_TOPIC,
    ZONE_Z_MAX,
    ZONE_Z_MIN,
    bresenham_line,
    quaternion_rotate_xy,
    sensor_qos,
    to_cell_index,
)

OCCUPIED = 100
FREE = 0
UNKNOWN = -1


class GlobalMapBuilderNode(Node):
    def __init__(self):
        super().__init__("global_map_builder_node")

        self.declare_parameter("global_resolution", GLOBAL_GRID_RESOLUTION)
        self.declare_parameter("global_size", GLOBAL_GRID_SIZE)
        self.declare_parameter("zone_z_min", ZONE_Z_MIN)
        self.declare_parameter("zone_z_max", ZONE_Z_MAX)
        self.declare_parameter("min_range", GLOBAL_MIN_RANGE)
        self.declare_parameter("max_range", GLOBAL_MAX_RANGE)

        resolution = self.get_parameter("global_resolution").value
        size_m = self.get_parameter("global_size").value
        self.cells = max(1, int(round(size_m / resolution)))
        self.grid = [UNKNOWN] * (self.cells * self.cells)

        # World pose (in the odom frame) of the grid's cell (0,0) corner;
        # fixed once we see the first odometry message, so the robot's
        # starting position ends up at the center of the grid.
        self.origin_x = None
        self.origin_y = None

        self.latest_odom = None  # (x, y, qx, qy, qz, qw)

        self.map_pub = self.create_publisher(OccupancyGrid, GLOBAL_MAP_TOPIC, 10)

        self.odom_sub = self.create_subscription(
            Odometry, LEG_ODOM_TOPIC, self.cb_odom, sensor_qos())
        self.lidar_sub = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.cb_pointcloud, sensor_qos())

        self.warned_no_odom = False

        self.get_logger().info(
            f"Global map builder started (perception only, no motion "
            f"commands). Fake-SLAM: uses {LEG_ODOM_TOPIC} for pose, no "
            f"loop closure, will drift on long runs. Grid: {size_m}m x "
            f"{size_m}m at {resolution}m/cell, centered on the robot's "
            f"start position."
        )

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = (p.x, p.y, q.x, q.y, q.z, q.w)

        if self.origin_x is None:
            resolution = self.get_parameter("global_resolution").value
            size_m = self.get_parameter("global_size").value
            self.origin_x = p.x - size_m / 2.0
            self.origin_y = p.y - size_m / 2.0
            self.get_logger().info(
                f"First odometry received, grid origin fixed at "
                f"({self.origin_x:.2f}, {self.origin_y:.2f}) in the "
                f"'{msg.header.frame_id}' frame."
            )

    def cb_pointcloud(self, msg: PointCloud2):
        if self.latest_odom is None:
            if not self.warned_no_odom:
                self.warned_no_odom = True
                self.get_logger().warn(
                    f"No message received yet on {LEG_ODOM_TOPIC} — "
                    "cannot place points in the world frame, skipping."
                )
            return

        resolution = self.get_parameter("global_resolution").value
        z_min = self.get_parameter("zone_z_min").value
        z_max = self.get_parameter("zone_z_max").value
        min_range = self.get_parameter("min_range").value
        max_range = self.get_parameter("max_range").value

        robot_x, robot_y, qx, qy, qz, qw = self.latest_odom
        robot_gx = to_cell_index(robot_x, self.origin_x, resolution)
        robot_gy = to_cell_index(robot_y, self.origin_y, resolution)

        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < z_min or z > z_max:
                continue
            r = (x * x + y * y) ** 0.5
            if r < min_range or r > max_range:
                continue

            dx, dy = quaternion_rotate_xy(qx, qy, qz, qw, x, y, z)
            world_x = robot_x + dx
            world_y = robot_y + dy

            gx = to_cell_index(world_x, self.origin_x, resolution)
            gy = to_cell_index(world_y, self.origin_y, resolution)
            if not (0 <= gx < self.cells and 0 <= gy < self.cells):
                continue

            for cx, cy in bresenham_line(robot_gx, robot_gy, gx, gy):
                if not (0 <= cx < self.cells and 0 <= cy < self.cells):
                    continue
                if cx == gx and cy == gy:
                    self.grid[cy * self.cells + cx] = OCCUPIED
                elif self.grid[cy * self.cells + cx] == UNKNOWN:
                    self.grid[cy * self.cells + cx] = FREE

        self.publish_map(msg, resolution)

    def publish_map(self, source_msg: PointCloud2, resolution: float):
        grid_msg = OccupancyGrid()
        grid_msg.header = source_msg.header
        grid_msg.header.frame_id = "leg_odom"
        grid_msg.info.resolution = float(resolution)
        grid_msg.info.width = self.cells
        grid_msg.info.height = self.cells
        origin = Pose()
        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        origin.orientation.w = 1.0
        grid_msg.info.origin = origin
        grid_msg.data = self.grid

        self.map_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapBuilderNode()
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
