#!/usr/bin/env python3
"""
One-shot snapshot saver for /x2/local_safety_map.

Waits for a single OccupancyGrid message, writes it to disk as a
standard ROS map (PGM image + YAML metadata, same convention as
nav2_map_server's map_saver_cli), then exits. Read-only / perception
only: never commands the robot.

Run:
  ros2 run x2_safety_toolkit map_snapshot_saver --ros-args -p output_path:=/home/agi/my_map
"""

import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from x2_safety_toolkit.common import LOCAL_SAFETY_MAP_TOPIC

OCCUPIED_PIXEL = 0
FREE_PIXEL = 254
UNKNOWN_PIXEL = 205


class MapSnapshotSaver(Node):
    def __init__(self):
        super().__init__("map_snapshot_saver")

        self.declare_parameter("output_path", "map_snapshot")
        self.saved = False

        self.sub = self.create_subscription(
            OccupancyGrid, LOCAL_SAFETY_MAP_TOPIC, self.cb_map, 10)

        self.get_logger().info(
            f"Waiting for one message on {LOCAL_SAFETY_MAP_TOPIC} to save..."
        )

    def cb_map(self, msg: OccupancyGrid):
        if self.saved:
            return
        self.saved = True

        output_path = self.get_parameter("output_path").value
        dirname = os.path.dirname(output_path)
        if dirname:
            os.makedirs(dirname, exist_ok=True)

        pgm_path = output_path + ".pgm"
        yaml_path = output_path + ".yaml"

        self.write_pgm(pgm_path, msg)
        self.write_yaml(yaml_path, os.path.basename(pgm_path), msg)

        self.get_logger().info(f"Saved snapshot: {pgm_path} and {yaml_path}")
        self.get_logger().info("Done, shutting down.")
        rclpy.shutdown()

    def write_pgm(self, path: str, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height

        def pixel_for(value: int) -> int:
            if value < 0:
                return UNKNOWN_PIXEL
            if value == 0:
                return FREE_PIXEL
            return OCCUPIED_PIXEL

        # PGM row 0 is the top of the image; nav2's map_saver convention
        # flips the grid vertically so row (height-1) of the grid (the
        # far edge, in our lateral-rows layout) is drawn at the top.
        rows = []
        for row in range(height - 1, -1, -1):
            base = row * width
            rows.append(bytes(
                pixel_for(msg.data[base + col]) for col in range(width)
            ))

        with open(path, "wb") as f:
            f.write(b"P5\n")
            f.write(b"# x2_safety_toolkit local safety map snapshot\n")
            f.write(f"{width} {height}\n".encode("ascii"))
            f.write(b"255\n")
            for row_bytes in rows:
                f.write(row_bytes)

    def write_yaml(self, path: str, image_filename: str, msg: OccupancyGrid):
        origin = msg.info.origin.position
        with open(path, "w") as f:
            f.write(f"image: {image_filename}\n")
            f.write(f"resolution: {msg.info.resolution}\n")
            f.write(f"origin: [{origin.x}, {origin.y}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")
            f.write(f"# frame_id: {msg.header.frame_id}\n")


def main(args=None):
    rclpy.init(args=args)
    node = MapSnapshotSaver()
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
