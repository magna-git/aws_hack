"""Tool : perception SOL/PROCHE — caméra RGB-D de tête (couleur + profondeur alignées), base64.

La caméra RGB-D (Orbbec Gemini335) donne couleur et profondeur du MÊME capteur, à la MÊME
résolution (1280×720) : elles sont registrées pixel-à-pixel par le firmware — aucune parallaxe,
aucune calibration. On superpose la profondeur en fausses couleurs (proche=rouge, loin=bleu)
sur la couleur, avec des labels de distance.

⚠️ La caméra RGB-D est montée INCLINÉE ~50° vers le bas ET tête-en-bas : elle cadre le sol et
le proche devant les pieds du robot. On tourne l'image de 180° pour la remettre à l'endroit.
Portée utile ~3.7 m (interaction proche). Pour la scène AVANT à distance, voir perceive_front.

Fichier PUR sans décorateur : les couches Strands/MCP l'enveloppent depuis le registre.
cv2/numpy présents sur le robot ; imports différés.
"""
from __future__ import annotations

from tools.robot_session import RobotUnavailableError, get_session, new_image_path

RGBD_COLOR_TOPIC = "/aima/hal/sensor/rgbd_head_front/rgb_image/compressed"
RGBD_DEPTH_TOPIC = "/aima/hal/sensor/rgbd_head_front/depth_image"


def _draw_labels(cv2, np, img, depth, cols: int, rows: int) -> None:
    """Grille clairsemée de labels : distance médiane par zone (mètres)."""
    h, w = img.shape[:2]
    dh, dw = depth.shape
    for r in range(rows):
        for c in range(cols):
            r0, r1 = r * dh // rows, max(r * dh // rows + 1, (r + 1) * dh // rows)
            c0, c1 = c * dw // cols, max(c * dw // cols + 1, (c + 1) * dw // cols)
            vals = depth[r0:r1, c0:c1]
            vals = vals[vals > 0.0]
            if vals.size == 0:
                continue
            text = f"{float(np.median(vals)):.1f}"
            px = int((c + 0.5) * w / cols)
            py = int((r + 0.5) * h / rows)
            scale = max(0.5, w / 1600.0)
            thick = max(1, int(scale * 2))
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
            org = (px - tw // 2, py + th // 2)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), thick, cv2.LINE_AA)


def perceive_ground(max_range: float = 4.0, alpha: float = 0.45) -> str:
    """Capture the ground/near view from the head RGB-D camera (color + aligned depth).

    Returns the color image with its pixel-aligned depth overlaid in false color (red = near,
    blue = far) plus a grid of distance labels in meters. The RGB-D camera points down/forward,
    so this shows the floor and nearby objects in front of the robot's feet (useful range
    ~3.7 m). Depth and color are from the same sensor, so alignment is pixel-perfect. The robot
    does not move (read-only). For the scene farther ahead, use perceive_front instead.

    Args:
        max_range (float): Distance in meters mapped to blue; farther saturates. Defaults 4.0.
        alpha (float): Opacity of the depth color layer over the photo [0..1]. Defaults 0.45.

    Returns:
        str: The path to the saved JPEG (a unique file on the robot, e.g.
            /tmp/x2_images/perceive_ground_<id>.jpg) on success, or a human-readable error
            message if the robot is unreachable or no data arrives in time.
    """
    try:
        max_range = float(max_range)
        alpha = float(alpha)
    except (TypeError, ValueError):
        return "Invalid arguments: max_range and alpha must be numbers."
    if max_range <= 0:
        return f"Invalid max_range={max_range}: must be positive."
    if not (0.0 <= alpha <= 1.0):
        return f"Invalid alpha={alpha}: must be within [0, 1]."

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    import cv2  # imports différés (robot)
    import numpy as np

    try:
        jpeg = session.capture_frame(RGBD_COLOR_TOPIC)
        depth = session.capture_depth(RGBD_DEPTH_TOPIC)
    except RobotUnavailableError as exc:
        return f"Could not capture RGB-D data: {exc}."

    photo = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if photo is None:
        return "Could not decode the RGB-D color JPEG."

    # Remettre à l'endroit (caméra montée tête-en-bas) : couleur ET depth de 180°.
    photo = cv2.rotate(photo, cv2.ROTATE_180)
    depth = cv2.rotate(depth, cv2.ROTATE_180)

    h, w = photo.shape[:2]
    # Si depth et couleur diffèrent en taille, aligner la depth sur la couleur.
    if depth.shape[:2] != (h, w):
        depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

    valid = depth > 0.0
    inv = (255 - (np.clip(depth, 0.0, max_range) / max_range * 255.0)).astype(np.uint8)
    colored = cv2.applyColorMap(inv, cv2.COLORMAP_JET)

    out = photo.copy()
    blend = cv2.addWeighted(photo, 1.0 - alpha, colored, alpha, 0.0)
    out[valid] = blend[valid]
    _draw_labels(cv2, np, out, depth, cols=8, rows=6)

    path = new_image_path("perceive_ground")
    if not cv2.imwrite(path, out):
        return f"Could not write the RGB-D overlay image to {path}."
    return path
