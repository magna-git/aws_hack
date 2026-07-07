#!/usr/bin/env python3
"""
Quick terminal (ASCII) viewer for /x2/local_safety_map.

Read-only debug tool: subscribes and prints, publishes nothing. Handy to
"see" the local safety map straight from an SSH session without rviz.

Legend: '#' occupied, '.' free, ' ' unknown. Robot is at the bottom
center, forward is up.

Run:
  ros2 run x2_safety_toolkit map_ascii_view
"""

import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from x2_safety_toolkit.common import LOCAL_SAFETY_MAP_TOPIC

CHAR_OCCUPIED = "#"
CHAR_FREE = "."
CHAR_UNKNOWN = " "


class MapAsciiView(Node):
    def __init__(self):
        super().__init__("map_ascii_view")
        self.sub = self.create_subscription(
            OccupancyGrid, LOCAL_SAFETY_MAP_TOPIC, self.cb_map, 10)
        self.get_logger().info(
            f"Subscribed (read-only) to {LOCAL_SAFETY_MAP_TOPIC}. "
            "Waiting for the first map..."
        )

    def cb_map(self, msg: OccupancyGrid):
        width = msg.info.width    # forward cells (mx)
        height = msg.info.height  # lateral cells (my): 0=right, height-1=left

        def char_for(cell):
            if cell == 100:
                return CHAR_OCCUPIED
            if cell == 0:
                return CHAR_FREE
            return CHAR_UNKNOWN

        lines = []
        # One row per forward distance (mx), far at top, robot (mx=0) at
        # bottom. Within a row: robot's left on screen-left, right on
        # screen-right (as if standing behind the robot looking forward).
        for mx in range(width - 1, -1, -1):
            row = "".join(
                char_for(msg.data[my * width + mx])
                for my in range(height - 1, -1, -1)
            )
            lines.append("|" + row + "|")

        os.system("clear")
        print(f"local safety map — resolution={msg.info.resolution:.2f}m "
              f"size={width}x{height} frame={msg.header.frame_id}")
        print("(far)")
        print("+" + "-" * height + "+")
        print("\n".join(lines))
        print("+" + "-" * height + "+")
        pad = max(0, (height - 5) // 2)
        print(" " * pad + "robot (near)")
        print(f"legend: {CHAR_OCCUPIED}=occupied  {CHAR_FREE}=free  "
              f"{CHAR_UNKNOWN}=unknown  |  left is on your left, "
              "right is on your right")


def main(args=None):
    rclpy.init(args=args)
    node = MapAsciiView()
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
