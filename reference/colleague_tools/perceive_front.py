"""Tool : perception AVANT — nuage LiDAR projeté sur la photo de la caméra avant, en base64.

Chaque point LiDAR (repère poitrine `lidar_chest_front`) est transformé vers le repère de la
caméra de tête `rgb_head_center` via la TF mesurée, puis projeté avec les intrinsèques K +
distorsion D de la caméra. La parallaxe poitrine↔tête (26.7 cm) est donc corrigée : les points
tombent au PIXEL EXACT de la photo. Points coloriés par distance (proche=rouge, loin=bleu).

Calibration figée (récupérée du robot 2026-07-07, cf. mémoire x2-lidar-camera-calibration).
On fige la TF STATIQUE plutôt que d'interroger /tf en direct : c'est du tf_static stable,
ça évite de monter un buffer TF, et c'est plus robuste.

Fichier PUR sans décorateur : les couches Strands/MCP l'enveloppent depuis le registre.
cv2/numpy/tf2 sont présents sur le robot ; imports différés pour que le fichier s'importe
même sans eux (laptop).
"""
from __future__ import annotations

from tools.robot_session import RobotUnavailableError, get_session, new_image_path

LIDAR_TOPIC = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
FRONT_CAMERA_TOPIC = "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed"

# Transfo homogène 4x4 : point dans lidar_chest_front -> dans rgb_head_center.
T_CAM_LIDAR = [
    [-0.003, 0.000, -1.000, -0.000],
    [-0.000, 1.000,  0.000,  0.267],
    [ 1.000, 0.000, -0.003,  0.026],
    [ 0.000, 0.000,  0.000,  1.000],
]
# Intrinsèques caméra avant rgb_head_front_center (2688×1944).
K = [[1441.4195887949, 0.0, 1352.2865561583],
     [0.0, 1441.4879927136, 918.6665684969],
     [0.0, 0.0, 1.0]]
# Distorsion 8 coeffs (rational_polynomial).
D = [0.6369801253, 0.0392898564, 9.51099e-05, 1.51925e-05,
     -0.0015346098, 1.0473919567, 0.2013679704, -0.002533797]


def perceive_front(max_range: float = 6.0, point_radius: int = 5) -> str:
    """Capture the scene AHEAD and overlay the chest LiDAR onto the front camera photo.

    Projects each LiDAR point onto the front head-camera image at its exact pixel (the
    26.7 cm chest↔head parallax is corrected via the robot's calibration), colored by
    distance (red = near, blue = far). Gives an accurate depth-annotated view of what is in
    front of the robot, out to a few meters. The robot does not move (read-only).

    Args:
        max_range (float): Distance in meters mapped to blue; farther points saturate.
            Defaults to 6.0. Lower it to spread colors over a closer scene.
        point_radius (int): Radius in pixels of each drawn LiDAR point. Defaults to 5.
            Larger = more visible points but hides more of the photo.

    Returns:
        str: The path to the saved JPEG overlay (a unique file on the robot, e.g.
            /tmp/x2_images/perceive_front_<id>.jpg) on success, or a human-readable error
            message if the robot is unreachable or no data arrives in time.
    """
    try:
        max_range = float(max_range)
        point_radius = int(point_radius)
    except (TypeError, ValueError):
        return "Invalid arguments: max_range must be a number and point_radius an integer."
    if max_range <= 0:
        return f"Invalid max_range={max_range}: must be positive."

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    import cv2  # imports différés (robot)
    import numpy as np

    # [1] Photo avant (JPEG) + nuage LiDAR brut.
    try:
        jpeg = session.capture_frame(FRONT_CAMERA_TOPIC)
        cloud = session.capture_pointcloud(LIDAR_TOPIC, frames=3).astype(np.float64)
    except RobotUnavailableError as exc:
        return f"Could not capture front perception data: {exc}."

    photo = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if photo is None:
        return "Could not decode the front camera JPEG."
    h, w = photo.shape[:2]

    T = np.array(T_CAM_LIDAR, dtype=np.float64)
    k = np.array(K, dtype=np.float64)
    d = np.array(D, dtype=np.float64)

    # [2] LiDAR -> caméra, garder les points devant la caméra (z>0 en repère optique).
    pts_h = np.hstack([cloud, np.ones((cloud.shape[0], 1))])
    cam = (T @ pts_h.T).T[:, :3]
    cam = cam[cam[:, 2] > 0.05]
    if cam.shape[0] == 0:
        return "No LiDAR points project in front of the camera (nothing ahead in range)."

    # [3] Projection K + D (distorsion incluse).
    img_pts, _ = cv2.projectPoints(cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, d)
    img_pts = img_pts.reshape(-1, 2)

    # [4] Couleur par distance ; tri loin->proche pour occlusion correcte au dessin.
    dist = np.linalg.norm(cam, axis=1)
    inv = (255 - (np.clip(dist, 0.0, max_range) / max_range * 255.0)).astype(np.uint8)
    colors = cv2.applyColorMap(inv.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    overlay = photo.copy()
    drawn = 0
    for i in np.argsort(-dist):
        u, v = img_pts[i]
        if 0 <= u < w and 0 <= v < h:
            c = colors[i]
            cv2.circle(overlay, (int(u), int(v)), point_radius,
                       (int(c[0]), int(c[1]), int(c[2])), -1, cv2.LINE_AA)
            drawn += 1
    out = cv2.addWeighted(overlay, 0.85, photo, 0.15, 0.0)

    path = new_image_path("perceive_front")
    if not cv2.imwrite(path, out):
        return f"Could not write the overlay image to {path}."
    return path
