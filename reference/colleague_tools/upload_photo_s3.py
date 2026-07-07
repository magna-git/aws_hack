"""Tool : capture une photo depuis la caméra de tête du X2 et l'uploade sur Amazon S3.

Fonction PURE (sans décorateur) : les couches Strands/MCP l'enveloppent depuis le registre.
Réutilise la logique de capture de `robot_session.capture_frame` (comme capture_photo.py),
puis pousse le JPEG sur S3 via boto3 et renvoie l'URI s3://.

⚠️ CRÉDENTIELS AWS : ce tool s'exécute LÀ OÙ TOURNE LE SERVEUR MCP, c'est-à-dire SUR LE
ROBOT (Jetson) — PAS sur le laptop qui porte l'agent. La clé Bedrock de l'agent ne traverse
donc PAS le lien SSH/stdio. Il faut fournir des credentials AWS + un accès internet CÔTÉ
ROBOT. Le plus simple : les exporter dans le shell qui lance le serveur MCP :

    export AWS_ACCESS_KEY_ID=...
    export AWS_SECRET_ACCESS_KEY=...
    export AWS_SESSION_TOKEN=...        # si credentials temporaires (Isengard)
    export AWS_REGION=us-east-1
    cd /tmp && python3 -m x2_mcp.server

boto3 lit ces variables automatiquement. À défaut, le tool renvoie un message d'erreur
lisible (il ne lève jamais) au lieu de planter la boucle de l'agent.
"""
from __future__ import annotations

import datetime
import os

from tools.robot_session import RobotUnavailableError, get_session

# Topic caméra tête avant (CompressedImage, déjà du JPEG) — identique à capture_photo.py.
CAMERA_TOPIC = "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed"

# Bucket S3 de destination (créé par la stack InspectionStack). Surchargeable au runtime via
# la variable d'environnement X2_PHOTO_BUCKET côté robot (elle a la priorité), ou l'argument
# bucket= à l'appel. Région : fallback us-east-1, surchargeable via AWS_REGION.
DEFAULT_BUCKET = os.environ.get(
    "X2_PHOTO_BUCKET", "inspectionstack-dataphotosbucket24e72d20-lewedtegnhfv")
AWS_REGION = os.environ.get("AWS_REGION", "us-east-1")


def capture_and_upload_photo(bucket: str = "", key_prefix: str = "x2-photos/") -> str:
    """Capture one photo from the X2's head camera and upload it to Amazon S3.

    Grabs a single JPEG frame from the front head RGB camera and uploads it to the given
    S3 bucket. The object key is built from the prefix plus a UTC timestamp, e.g.
    "x2-photos/x2-20260707T160600Z.jpg".

    Args:
        bucket: Destination S3 bucket name. If empty, uses the X2_PHOTO_BUCKET env var
            (or the module default). Provide this to override at call time.
        key_prefix: Key prefix (folder) for the object. Defaults to "x2-photos/".

    Returns:
        str: On success, the "s3://bucket/key" URI of the uploaded photo. On failure, a
            human-readable message (robot unreachable, no frame, missing AWS credentials,
            or upload error). This function never raises.
    """
    target_bucket = bucket or DEFAULT_BUCKET
    if not target_bucket or target_bucket == "REPLACE_ME_x2-photos-bucket":
        return (
            "No S3 bucket configured. Set the X2_PHOTO_BUCKET environment variable on the "
            "robot (or pass bucket=...), or edit DEFAULT_BUCKET in tools/upload_photo_s3.py."
        )

    # 1. Capture the frame (same path as capture_photo).
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

    # 2. Upload to S3. Deferred boto3 import so this module lints without boto3 installed.
    timestamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    key = f"{key_prefix}x2-{timestamp}.jpg"

    try:
        import boto3  # import différé
    except ImportError:
        return (
            "boto3 is not installed on the robot. Install it in the ROS2 python "
            "(python3 -m pip install --user boto3), then retry."
        )

    try:
        s3 = boto3.client("s3", region_name=AWS_REGION)
        s3.put_object(
            Bucket=target_bucket,
            Key=key,
            Body=jpeg,
            ContentType="image/jpeg",
        )
    except Exception as exc:  # noqa: BLE001 — never let the agent loop crash.
        return (
            f"Photo captured ({len(jpeg)} bytes) but the S3 upload failed: {exc}. "
            "Are AWS credentials available on the robot (AWS_ACCESS_KEY_ID / "
            "AWS_SECRET_ACCESS_KEY / AWS_SESSION_TOKEN) and does the Jetson have internet "
            f"egress to bucket '{target_bucket}' in region '{AWS_REGION}'?"
        )

    return f"Photo uploaded to s3://{target_bucket}/{key}"
