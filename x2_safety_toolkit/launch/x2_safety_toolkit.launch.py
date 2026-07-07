"""
Launch file for x2_safety_toolkit — pick which nodes to bring up by
commenting/uncommenting lines below, instead of passing launch arguments.

To disable a node: comment out its Node(...) entry in the `nodes` list.
To enable one: uncomment it. That's it.

safe_velocity_controller_node keeps dry_run:=true by default even when
enabled below — flip its `parameters` line to dry_run:=false only when
the robot is in Stable Standing Mode and someone is ready to stop it.

Run:
  ros2 launch x2_safety_toolkit x2_safety_toolkit.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [
        Node(
            package="x2_safety_toolkit",
            executable="lidar_inspector_node",
            name="lidar_inspector_node",
            output="screen",
        ),
        Node(
            package="x2_safety_toolkit",
            executable="obstacle_detector_node",
            name="obstacle_detector_node",
            output="screen",
        ),
        Node(
            package="x2_safety_toolkit",
            executable="local_safety_map_node",
            name="local_safety_map_node",
            output="screen",
        ),
        # Node(
        #     package="x2_safety_toolkit",
        #     executable="global_map_builder_node",
        #     name="global_map_builder_node",
        #     output="screen",
        # ),
        # Node(
        #     package="x2_safety_toolkit",
        #     executable="safe_velocity_controller_node",
        #     name="safe_velocity_controller_node",
        #     output="screen",
        #     parameters=[{"dry_run": True}],
        # ),
    ]

    return LaunchDescription(nodes)
