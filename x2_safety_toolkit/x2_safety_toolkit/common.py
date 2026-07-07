"""Shared constants and helpers for the x2_safety_toolkit package.

All nodes in this package are perception/safety building blocks for the
AgiBot X2 chest-front LiDAR, used as a stand-in for the (unavailable)
official SLAM stack. Nothing here ever moves the robot on its own:
only safe_velocity_controller_node can publish to the motion-control
velocity topic, and only when explicitly run with dry_run:=false.
"""

import json

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

LIDAR_TOPIC = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
VELOCITY_TOPIC = "/aima/mc/locomotion/velocity"
LEG_ODOM_TOPIC = "/aima/mc/leg_odometry"

OBSTACLE_STATUS_TOPIC = "/x2/obstacle_status"
LOCAL_SAFETY_MAP_TOPIC = "/x2/local_safety_map"
DEBUG_POINTS_TOPIC = "/x2/debug_points"
GLOBAL_MAP_TOPIC = "/x2/global_map"

# Default global map geometry, meters / meters-per-cell.
GLOBAL_GRID_RESOLUTION = 0.10
GLOBAL_GRID_SIZE = 20.0  # square: 20m x 20m, robot's start pose at the center

# Default range gate for the global map (meters), to reject self-body hits
# (too close) and noisy far returns.
GLOBAL_MIN_RANGE = 0.15
GLOBAL_MAX_RANGE = 5.0

# Default frontal detection zone (LiDAR frame: x forward, y left, z up), meters.
ZONE_X_MIN = 0.15
ZONE_X_MAX = 2.0
ZONE_Y_MIN = -0.45
ZONE_Y_MAX = 0.45
ZONE_Z_MIN = -0.60
ZONE_Z_MAX = 0.80

# Default distance thresholds, meters.
FRONT_STOP_DISTANCE = 0.8
FRONT_SLOW_DISTANCE = 1.2

# Default safety speed limits.
FORWARD_SPEED = 0.20
SLOW_SPEED = 0.10
TURN_SPEED = 0.25

# Default local safety map geometry, meters / meters-per-cell.
GRID_RESOLUTION = 0.10
GRID_WIDTH = 4.0   # lateral extent (left-right)
GRID_DEPTH = 4.0   # forward extent

STATE_CLEAR = "CLEAR"
STATE_SLOW = "SLOW"
STATE_STOP = "STOP"
STATE_UNKNOWN = "UNKNOWN"


def sensor_qos(depth: int = 5) -> QoSProfile:
    """Same profile as the official echo_lidar_data.py example."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def in_zone(x, y, z, x_min, x_max, y_min, y_max, z_min, z_max) -> bool:
    return x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max


def classify_distance(distance, stop_distance, slow_distance) -> str:
    if distance is None:
        return STATE_UNKNOWN
    if distance < stop_distance:
        return STATE_STOP
    if distance < slow_distance:
        return STATE_SLOW
    return STATE_CLEAR


def build_obstacle_status_json(
    stamp_sec: float,
    frame_id: str,
    min_front_distance,
    left_clearance,
    right_clearance,
    state: str,
) -> str:
    return json.dumps({
        "stamp": stamp_sec,
        "frame_id": frame_id,
        "min_front_distance": min_front_distance,
        "left_clearance": left_clearance,
        "right_clearance": right_clearance,
        "state": state,
    })


def parse_obstacle_status_json(data: str) -> dict:
    return json.loads(data)


def quaternion_rotate_xy(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector (vx,vy,vz) by quaternion (qx,qy,qz,qw); return (x,y).

    Only x,y are returned since every grid in this package is a flat 2D
    projection; z is used solely for the height-band filters upstream.
    """
    # v' = v + qw*t + q_xyz x t, with t = 2*(q_xyz x v).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    return (rx, ry)


def to_cell_index(value: float, origin: float, resolution: float) -> int:
    """floor((value-origin)/resolution), nudged by an epsilon so a value
    that should land exactly on a cell boundary isn't truncated into the
    wrong cell by ordinary float rounding noise."""
    return int((value - origin) / resolution + 1e-6)


def bresenham_line(x0, y0, x1, y1):
    """Integer grid cells from (x0,y0) to (x1,y1), inclusive of both ends."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
