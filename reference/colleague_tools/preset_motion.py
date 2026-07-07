"""Tools Strands : mouvements pré-programmés (presets) du X2.

- `list_preset_motions()` : renvoie le catalogue des gestes disponibles (pour que l'agent
  découvre ce qu'il peut demander, au lieu de deviner).
- `preset_motion(motion, arm, interrupt)` : joue un preset via le service SetMcPresetMotion.

Le message `aimdk_msgs/msg/McPresetMotion` définit 27 constantes, mais seule la famille
1000 (gestes de bras) est réellement jouable sur notre unité — voir la note sur PRESETS.
Ce fichier n'a AUCUN décorateur et est pur : les couches Strands/MCP l'enveloppent depuis
le registre. rclpy/aimdk sont derrière robot_session, donc ce fichier s'importe même sans ROS2.
"""
from __future__ import annotations

import time

from tools.robot_session import RobotUnavailableError, get_session

# Service preset (forme URL-encodée : _5F = underscore).
PRESET_SRV = "/aimdk_5Fmsgs/srv/SetMcPresetMotion"

AREA_IDS = {"left": 1, "right": 2}

# Catalogue des presets : nom convivial -> (id fabricant, catégorie, description).
# Les ids proviennent des constantes de aimdk_msgs/msg/McPresetMotion sur le robot.
# `arm_specific=True` = le paramètre `arm` a du sens ; False = geste global (bras ignoré).
#
# ⚠️ Vérifié sur le robot (2026-07-07) : SEULE la famille 1000 (gestes de bras) est
# réellement jouable via SetMcPresetMotion sur cette unité. Les familles 3000
# (interactions : thumbs_up, dance…) et 4000 (tête : nod…) sont définies dans le message
# mais REFUSÉES par le robot (code=1, state=2 FAILURE) quels que soient le mode et l'area.
# On ne liste donc que la famille 1000.
PRESETS: dict[str, dict] = {
    "raise":     {"id": 1001, "category": "arm", "arm_specific": True,  "desc": "raise the hand"},
    "wave":      {"id": 1002, "category": "arm", "arm_specific": True,  "desc": "wave the hand"},
    "handshake": {"id": 1003, "category": "arm", "arm_specific": True,  "desc": "offer a handshake"},
    "airkiss":   {"id": 1004, "category": "arm", "arm_specific": True,  "desc": "blow a kiss"},
    "clap":      {"id": 1008, "category": "arm", "arm_specific": False, "desc": "clap hands"},
    "fistbump":  {"id": 1009, "category": "arm", "arm_specific": True,  "desc": "fist bump"},
    "salute":    {"id": 1013, "category": "arm", "arm_specific": True,  "desc": "military salute"},
}

# Alias tolérants : le LLM peut passer des formulations libres -> clé du catalogue.
_MOTION_ALIASES = {
    "raise arm": "raise", "raise_arm": "raise", "raise hand": "raise", "lift": "raise",
    "air kiss": "airkiss", "air-kiss": "airkiss", "kiss": "airkiss",
    "shake": "handshake", "shake hands": "handshake", "hand shake": "handshake",
    "shake hand": "handshake",
    "fist bump": "fistbump", "fist_bump": "fistbump", "bump": "fistbump",
    "clap hands": "clap", "applaud": "clap",
}


def list_preset_motions() -> str:
    """List the X2 humanoid's built-in preset gestures that `preset_motion` can play.

    Use this to discover the available gestures before calling `preset_motion`. Returns each
    gesture's name, what it does, and whether the `arm` parameter is meaningful for it.

    Returns:
        str: A readable list of preset gesture names.
    """
    lines = [
        f"The X2 has {len(PRESETS)} preset arm gestures. "
        "Call preset_motion(motion=<name>, arm=<left|right>).",
        "",
    ]
    for name, meta in PRESETS.items():
        arm_note = "" if meta["arm_specific"] else "  (arm ignored — both hands)"
        lines.append(f"  - {name}: {meta['desc']}{arm_note}")
    return "\n".join(lines)


def preset_motion(motion: str, arm: str = "right", interrupt: bool = True) -> str:
    """Make the X2 humanoid perform a built-in arm gesture (wave, handshake, salute, clap...).

    Sets the robot to STAND_DEFAULT for stability, then triggers the manufacturer preset
    motion. These are safe, fluid factory gestures — not raw joint control. Call
    `list_preset_motions` first to see all available gesture names.

    Args:
        motion (str): Gesture name from the catalogue (e.g. "wave", "handshake", "salute",
            "clap", "raise"). See `list_preset_motions` for the full list.
        arm (str): Which arm — "left" or "right". Defaults to "right". Ignored for
            two-handed gestures (clap).
        interrupt (bool): Interrupt any preset still in progress and run this one. Defaults
            to True — the robot stays "busy" (state RUNNING) after a gesture and rejects a
            new one when interrupt=False, so chained gestures need interrupt=True.

    Returns:
        str: A human-readable success or failure message describing what happened.
    """
    # Normalisation + alias.
    motion_key = str(motion).strip().lower()
    motion_key = _MOTION_ALIASES.get(motion_key, motion_key)
    arm_key = str(arm).strip().lower()

    # Validation → on RENVOIE une chaîne (on ne lève pas) pour que l'agent se corrige.
    if motion_key not in PRESETS:
        return (
            f"Invalid motion '{motion}'. Call list_preset_motions to see the available "
            f"gesture names."
        )
    if arm_key not in AREA_IDS:
        return f"Invalid arm '{arm}'. Choose 'left' or 'right'."

    meta = PRESETS[motion_key]
    motion_id = meta["id"]
    area_id = AREA_IDS[arm_key]

    # Session partagée.
    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    # Sécurité : robot debout stable avant un geste.
    if not session.set_mode("STAND_DEFAULT"):
        return "Aborted: the robot did not accept STAND_DEFAULT mode; no motion was sent."
    time.sleep(1.0)  # laisser le contrôleur stabiliser la posture (cf. raise_arm.py)

    # Appel du preset.
    from aimdk_msgs.msg import CommonState  # import différé (dispo seulement sur le robot)
    from aimdk_msgs.srv import SetMcPresetMotion

    client = session.get_client(SetMcPresetMotion, PRESET_SRV)
    req = SetMcPresetMotion.Request()
    req.area.value = area_id
    req.motion.value = motion_id
    req.interrupt = bool(interrupt)

    try:
        result = session.call_service(client, req, service_name=PRESET_SRV)
    except RobotUnavailableError:
        return (
            f"The robot's motion service ({PRESET_SRV}) did not respond; "
            f"the '{motion_key}' gesture was not performed."
        )

    resp = result.response
    where = f"{arm_key} arm " if meta["arm_specific"] else ""
    if resp.header.code == 0 or resp.state.value == CommonState.RUNNING:
        return f"Done — the robot performed the '{motion_key}' gesture {where}(task {resp.task_id}).".replace("  ", " ")
    return (
        f"The '{motion_key}' gesture was rejected by the robot "
        f"(code={resp.header.code}, state={resp.state.value})."
    )
