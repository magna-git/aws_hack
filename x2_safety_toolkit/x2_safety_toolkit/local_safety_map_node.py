#!/usr/bin/env python3
"""
Local 2D safety map built from the chest-front LiDAR.

Projects the frontal point cloud into a small 2D occupancy grid in front
of the robot. Read-only / perception-only: never commands the robot.

Grid convention (in the LiDAR frame: x forward, y left):
  - cell (mx=0, my=grid_height/2) is the robot's position.
  - mx grows with forward distance (0..grid_depth).
  - my grows with lateral position from right (-width/2) to left (+width/2).
  - occupied = 100, unknown = -1, free (approximate) = 0.

Publishes:
  /x2/local_safety_map  (nav_msgs/OccupancyGrid)

Run:
  ros2 run x2_safety_toolkit local_safety_map_node
"""

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from x2_safety_toolkit.common import (
    GRID_DEPTH,
    GRID_RESOLUTION,
    GRID_WIDTH,
    LIDAR_TOPIC,
    LOCAL_SAFETY_MAP_TOPIC,
    ZONE_Z_MAX,
    ZONE_Z_MIN,
    sensor_qos,
    to_cell_index,
)

OCCUPIED = 100
FREE = 0
UNKNOWN = -1


class LocalSafetyMapNode(Node):
    def __init__(self):
        super().__init__("local_safety_map_node")

        self.declare_parameter("grid_resolution", GRID_RESOLUTION)
        self.declare_parameter("grid_width", GRID_WIDTH)
        self.declare_parameter("grid_depth", GRID_DEPTH)
        self.declare_parameter("zone_z_min", ZONE_Z_MIN)
        self.declare_parameter("zone_z_max", ZONE_Z_MAX)

        self.map_pub = self.create_publisher(
            OccupancyGrid, LOCAL_SAFETY_MAP_TOPIC, 10)

        self.sub = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.cb_pointcloud, sensor_qos())

        self.get_logger().info(
            f"Local safety map node started (perception only, no motion "
            f"commands). Subscribed to {LIDAR_TOPIC}, publishing "
            f"{LOCAL_SAFETY_MAP_TOPIC}."
        )

    def cb_pointcloud(self, msg: PointCloud2):
        resolution = self.get_parameter("grid_resolution").value
        width_m = self.get_parameter("grid_width").value
        depth_m = self.get_parameter("grid_depth").value
        z_min = self.get_parameter("zone_z_min").value
        z_max = self.get_parameter("zone_z_max").value

        num_x = max(1, int(round(depth_m / resolution)))   # forward cells
        num_y = max(1, int(round(width_m / resolution)))   # lateral cells
        y_min = -width_m / 2.0

        grid = [UNKNOWN] * (num_x * num_y)
        nearest_mx_per_my = [None] * num_y

        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < z_min or z > z_max:
                continue
            if x < 0.0 or x >= depth_m:
                continue
            if y < y_min or y >= y_min + width_m:
                continue

            mx = to_cell_index(x, 0.0, resolution)
            my = to_cell_index(y, y_min, resolution)
            if mx >= num_x or my >= num_y:
                continue

            grid[my * num_x + mx] = OCCUPIED
            if nearest_mx_per_my[my] is None or mx < nearest_mx_per_my[my]:
                nearest_mx_per_my[my] = mx

        # Approximate free space: cells strictly closer than the nearest
        # occupied cell in the same lateral column are known to be clear.
        for my in range(num_y):
            nearest_mx = nearest_mx_per_my[my]
            if nearest_mx is None:
                continue
            base = my * num_x
            for mx in range(nearest_mx):
                if grid[base + mx] == UNKNOWN:
                    grid[base + mx] = FREE

        grid_msg = OccupancyGrid()
        grid_msg.header = msg.header
        grid_msg.info.resolution = float(resolution)
        grid_msg.info.width = num_x
        grid_msg.info.height = num_y
        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = y_min
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        grid_msg.info.origin = origin
        grid_msg.data = grid

        self.map_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalSafetyMapNode()
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
