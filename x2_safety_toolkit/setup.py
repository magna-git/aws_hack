import glob

from setuptools import find_packages, setup

package_name = "x2_safety_toolkit"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description=(
        "X2 Perception Safety Toolkit: chest-front LiDAR inspection, "
        "obstacle detection, local 2D safety map and a dry-run-first "
        "safe velocity controller, used in place of the unavailable "
        "official SLAM stack."
    ),
    license="TODO: License declaration",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_inspector_node = x2_safety_toolkit.lidar_inspector_node:main",
            "obstacle_detector_node = x2_safety_toolkit.obstacle_detector_node:main",
            "local_safety_map_node = x2_safety_toolkit.local_safety_map_node:main",
            "safe_velocity_controller_node = x2_safety_toolkit.safe_velocity_controller_node:main",
            "map_ascii_view = x2_safety_toolkit.map_ascii_view:main",
            "map_snapshot_saver = x2_safety_toolkit.map_snapshot_saver:main",
            "global_map_builder_node = x2_safety_toolkit.global_map_builder_node:main",
        ],
    },
)
