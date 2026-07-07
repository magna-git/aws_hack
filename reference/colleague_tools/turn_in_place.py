"""Tool Strands/MCP : faire pivoter le X2 sur lui-même par crans de 45° (rotation debout).

Même séquence de locomotion que walk_distance.py (LOCOMOTION_DEFAULT -> SetMcInputSource(ADD)
-> publier une vitesse CONSTANTE @50 Hz -> zéros -> DELETE), mais on commande
`angular_velocity`. Le robot tourne SUR PLACE, typiquement pour orienter la caméra de tête
sous plusieurs angles avant de photographier.

⚠️ POURQUOI DES CRANS DE 45° FIXES (et pas un angle libre) — mesuré sur le robot 2026-07-07 :
la rotation est FORTEMENT non-linéaire et aucun capteur de yaw embarqué n'est fiable ici.
  - `/aima/mc/leg_odometry` : quaternion très incliné -> yaw = bruit pur (a rapporté 54°
    alors que le robot n'avait quasi pas bougé). Inutilisable.
  - gyro torse intégré : diverge du réel dès que le robot fait des pas. Inutilisable.
  - vitesse : 0.4 rad/s -> ~0° ; 1.0 rad/s -> 3 TOURS incontrôlés. Pas de proportionnalité.
  - durée à 0.6 rad/s : 1.5 s -> ~10°, 3.0 s -> ~45° (doubler la durée quadruple l'angle :
    le gait monte en régime). Donc calibrer par la durée est un piège.
La SEULE primitive fiable et répétable trouvée : UNE pulse `PULSE_SPEED` × `PULSE_DURATION`
suivie d'un arrêt franc = ~45° propre, dans les deux sens. On n'expose donc QUE ça, en
l'enchaînant pour des multiples de 45°.

Fichier pur (aucun décorateur) : les couches Strands/MCP l'enveloppent via le registre.
rclpy/aimdk sont derrière robot_session, donc ce fichier s'importe même sans ROS2.
"""
from __future__ import annotations

import time

from tools.robot_session import (
    INPUTACTION_ADD,
    INPUTACTION_DELETE,
    RobotUnavailableError,
    get_session,
)

VELOCITY_TOPIC = "/aima/mc/locomotion/velocity"
SRC_NAME = "node"
LOCOMOTION_MODE = "LOCOMOTION_DEFAULT"
STAND_MODE = "STAND_DEFAULT"

# Temps de stabilisation après un changement de mode (le contrôleur doit converger avant
# qu'on lui envoie des vitesses / relance une pulse). Mesuré empiriquement.
MODE_SETTLE_SECONDS = 1.0
# Après une rotation, le contrôleur refuse de re-basculer en LOCOMOTION s'il n'a pas fini
# de converger en STAND. On lui laisse plus de temps avant le cran suivant.
STAND_SETTLE_SECONDS = 3.0

# Primitives calibrées sur le robot par photos avant/après (2026-07-07). Une pulse à
# PULSE_SPEED pendant la durée donnée, suivie d'un arrêt franc, tourne de l'angle indiqué.
# Ne PAS toucher sans recalibrer : l'angle n'est PAS linéaire en la durée — il croît ~en
# durée² (le gait monte en régime). Mesuré : 1.5s→~10°, 2.0s→~18°, 3.0s→~45°.
PULSE_SPEED = 0.6          # rad/s (0.4 ne tourne pas, 1.0 part en vrille)

# Deux tailles de cran. "coarse" = gros cran (~45°), "fine" = petit cran (~18°) pour
# orienter la caméra plus finement. (durée en s, angle réel observé en °).
STEP_SIZES = {
    "coarse": (3.0, 45.0),
    "fine": (2.0, 18.0),
}
DEFAULT_STEP_SIZE = "coarse"

# Arrêt franc PROLONGÉ entre deux pulses : indispensable, sinon l'inertie de rotation
# fait sur-tourner (le sur-tour observé à 1.0 rad/s venait d'un arrêt trop bref).
STOP_SECONDS = 2.0

PUBLISH_RATE_HZ = 50.0

# Garde-fou : nombre max de crans par appel (8 × 45° = 360°). Enchaîner si besoin.
MAX_STEPS = 8


def turn_in_place(direction: str = "right", steps: int = 1,
                  step_size: str = DEFAULT_STEP_SIZE) -> str:
    """Pivot the X2 humanoid in place, in fixed angular increments, to point the head camera
    at a different direction (e.g. before taking a photo).

    Turning is OPEN-LOOP on this unit (no reliable yaw sensor / no SLAM), so only a few
    calibrated pulse sizes are exposed rather than an arbitrary angle. Pick a `step_size`:
    "coarse" turns ~45° per step, "fine" turns ~18° per step (for finer aiming). Ask for
    `steps` increments in the given `direction`. A firm stop is enforced between steps and at
    the end. Angles are empirical (±a few degrees), not guaranteed.

    ⚠️ MOVES A HUMANOID (rotates in place). Only call with the robot standing, a clear area
    around it, and someone ready to trigger the emergency stop.

    Args:
        direction (str): "left" (counter-clockwise) or "right" (clockwise). Defaults "right".
        steps (int): Number of increments to turn. Must be between 1 and 8. With
            step_size="coarse": 1 = ~45°, 2 = ~90°, 4 = ~180°. With step_size="fine":
            1 = ~18°, 2 = ~36°. Defaults to 1.
        step_size (str): "coarse" (~45° per step) or "fine" (~18° per step). Defaults to
            "coarse". Use "fine" to aim the camera more precisely.

    Returns:
        str: A human-readable message: total angle turned and direction, or why it was
            refused.
    """
    # Validation → on RENVOIE une chaîne (on ne lève pas) pour que l'agent se corrige.
    dir_key = str(direction).strip().lower()
    if dir_key in ("l", "ccw", "counterclockwise", "counter-clockwise"):
        dir_key = "left"
    if dir_key in ("r", "cw", "clockwise"):
        dir_key = "right"
    if dir_key not in ("left", "right"):
        return f"Invalid direction '{direction}'. Choose 'left' or 'right'."

    size_key = str(step_size).strip().lower()
    if size_key not in STEP_SIZES:
        return (
            f"Invalid step_size '{step_size}'. Choose "
            f"{' or '.join(repr(k) for k in STEP_SIZES)}."
        )
    pulse_duration, degrees_per_step = STEP_SIZES[size_key]

    try:
        steps = int(steps)
    except (TypeError, ValueError):
        return f"Invalid steps '{steps}': must be a whole number of increments."
    if steps < 1:
        return f"Invalid steps={steps}: must be at least 1 increment."
    if steps > MAX_STEPS:
        return (
            f"Refused steps={steps}: max {MAX_STEPS} increments per call "
            f"(~{int(MAX_STEPS * degrees_per_step)}° at step_size='{size_key}'). "
            "Call again to turn further."
        )

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    from aimdk_msgs.msg import McLocomotionVelocity  # import différé

    vel_pub = session.get_publisher(McLocomotionVelocity, VELOCITY_TOPIC, depth=10)

    # left = angular_velocity > 0 (CCW) ; right = < 0. Convention keyboard.cpp (Q/E).
    ang_cmd = PULSE_SPEED if dir_key == "left" else -PULSE_SPEED

    def publish_ang(ang: float) -> None:
        m = McLocomotionVelocity()
        m.header.stamp = session.node.get_clock().now().to_msg()
        m.source = SRC_NAME
        m.forward_velocity = 0.0
        m.lateral_velocity = 0.0
        m.angular_velocity = float(ang)
        vel_pub.publish(m)

    dt = 1.0 / PUBLISH_RATE_HZ
    pulse_frames = int(pulse_duration * PUBLISH_RATE_HZ)
    stop_frames = int(STOP_SECONDS * PUBLISH_RATE_HZ)

    def hold(ang: float, frames: int) -> None:
        for _ in range(frames):
            publish_ang(ang)
            session.spin_once(timeout_sec=0.0)
            time.sleep(dt)

    # Chaque cran est une séquence AUTONOME : STAND (re-stabilise le gait) -> LOCOMOTION ->
    # source ADD -> pulse -> arrêt franc -> source DELETE. Enchaîner les pulses SANS repasser
    # par STAND ne marche pas : après une rotation le contrôleur refuse LOCOMOTION ou "se
    # pose" et la pulse suivante ne tourne pas (diagnostic photos 2026-07-07). Le retour en
    # STAND entre les crans recrée les conditions d'un premier cran propre.
    done = 0
    try:
        for i in range(steps):
            # [1] posture stable avant CE cran (indispensable pour ré-accepter LOCOMOTION).
            # Après le 1er cran, laisser plus de temps : le contrôleur doit finir de converger
            # en STAND sinon il refuse LOCOMOTION au cran suivant.
            if not session.set_mode(STAND_MODE):
                break
            time.sleep(STAND_SETTLE_SECONDS if i > 0 else MODE_SETTLE_SECONDS)
            # [2] mode locomotion
            if not session.set_mode(LOCOMOTION_MODE):
                break
            time.sleep(MODE_SETTLE_SECONDS)
            # [3] source (sinon la MC ignore les vitesses)
            if not session.set_source(INPUTACTION_ADD):
                break
            # [4] la pulse calibrée, puis arrêt franc
            hold(ang_cmd, pulse_frames)
            hold(0.0, stop_frames)
            # [5] libère la source avant de rebasculer en STAND au tour suivant
            session.set_source(INPUTACTION_DELETE)
            done += 1
    finally:
        # arrêt garanti + posture stable finale (robot laissé debout, source libérée)
        hold(0.0, stop_frames)
        session.set_source(INPUTACTION_DELETE)
        session.set_mode(STAND_MODE)

    if done == 0:
        return (
            f"Aborted: the robot did not accept the required mode switch before any step; "
            "it did not move."
        )

    total = int(done * degrees_per_step)
    msg = (
        f"Done — turned ~{total}° to the {dir_key} in place "
        f"({done} × ~{int(degrees_per_step)}° '{size_key}' steps)."
    )
    if done < steps:
        msg += f" (Stopped early: only {done}/{steps} steps completed — mode switch refused.)"
    return msg
