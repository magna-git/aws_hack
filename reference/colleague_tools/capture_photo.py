"""Tool : capture une photo depuis la caméra de tête du X2 et l'écrit en JPEG sur le robot.

Fonction PURE (sans décorateur) : les couches Strands/MCP l'enveloppent depuis le registre.
La logique de capture vit dans robot_session.capture_frame ; ici on écrit le JPEG dans un
fichier unique et on renvoie son chemin — même convention que perceive_front/perceive_ground
(cf. commit 5daf922), pour ne pas inonder le canal MCP de base64.

Topic caméra validé (2026-07-07) : publie spontanément à 30 Hz, JPEG 2688x1944.
"""
from __future__ import annotations

from tools.robot_session import RobotUnavailableError, get_session, new_image_path

# Topic caméra tête avant (CompressedImage, déjà du JPEG).
CAMERA_TOPIC = "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed"


def capture_photo() -> str:
    """Capture one photo from the X2's head camera and save it as a JPEG on the robot.

    Grabs a single frame from the front head RGB camera and writes its raw JPEG bytes to a
    unique file. The robot does not move (read-only).

    Returns:
        str: The path to the saved JPEG (a unique file on the robot, e.g.
            /tmp/x2_images/capture_photo_<id>.jpg) on success, or a human-readable error
            message if the robot is unreachable or no camera frame arrives in time.
    """
    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    try:
        jpeg = session.capture_frame(CAMERA_TOPIC)
    except RobotUnavailableError as exc:
        return f"Could not capture a photo: {exc}."

    path = new_image_path("capture_photo")
    with open(path, "wb") as f:
        f.write(jpeg)
    return path
