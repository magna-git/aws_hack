"""Tool Strands/MCP : faire avancer le X2 d'une distance donnée, avec arrêt sur obstacle.

Combine trois briques déjà éprouvées du repo :
  1. La séquence de marche QUI MARCHE (x2_ros2/walk_forward.py) : LOCOMOTION_DEFAULT ->
     SetMcInputSource(ADD) -> publier une vitesse CONSTANTE >= 0.2 m/s @50 Hz -> zéros ->
     DELETE. PAS de rampe (la rampe passait sous le seuil 0.2 et faisait sursauter le robot).
  2. La lecture LiDAR frontal (py_examples/obstacle_avoidance_node.py) : on filtre une zone
     frontale du pointcloud et on garde la distance la plus proche.
  3. Le contrôle de distance en BOUCLE OUVERTE : distance ≈ vitesse × durée (pas d'odométrie).

L'arrêt sur obstacle est PRIORITAIRE sur la distance : dès qu'un point passe sous
STOP_DISTANCE dans la zone frontale, on coupe la vitesse et on rapporte "BLOCKED".

Fichier pur (aucun décorateur) : les couches Strands/MCP l'enveloppent via le registre.
rclpy/aimdk sont derrière robot_session, donc ce fichier s'importe même sans ROS2.
"""
from __future__ import annotations

import math
import time

from tools.robot_session import (
    INPUTACTION_ADD,
    INPUTACTION_DELETE,
    RobotUnavailableError,
    get_session,
)

LIDAR_TOPIC = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
VELOCITY_TOPIC = "/aima/mc/locomotion/velocity"
ODOM_TOPIC = "/aima/mc/leg_odometry"  # nav_msgs/Odometry, repère leg_odom (fixe monde)
SRC_NAME = "node"
LOCOMOTION_MODE = "LOCOMOTION_DEFAULT"

# Seuils du contrôleur (mc_locomotion_velocity.cpp) : une vitesse non nulle DOIT être
# dans [0.2 ; 1.0] m/s, sinon le robot saccade au lieu de marcher.
MIN_FWD_SPEED = 0.2
MAX_FWD_SPEED = 1.0

# Garde-fou dur : distance max autorisée par appel. La marche est en boucle ouverte
# (dérive ±15-20 %) sur un humanoïde ; on plafonne pour éviter un déplacement incontrôlé.
# L'arrêt sur obstacle LiDAR reste le premier filet de sécurité.
MAX_DISTANCE = 3.0

# Zone de détection frontale (repère LiDAR), en mètres — cf. obstacle_avoidance_node.py.
ZONE_X_MIN, ZONE_X_MAX = 0.15, 2.0
ZONE_Y_MIN, ZONE_Y_MAX = -0.45, 0.45
ZONE_Z_MIN, ZONE_Z_MAX = -0.60, 0.80

PUBLISH_RATE_HZ = 50.0
STOP_SETTLE_STEPS = 15  # nombre de trames de zéros publiées à l'arrêt

# Anti-bruit : on ne considère un obstacle proche que s'il est vu par AU MOINS ce nombre de
# points. Un point aberrant isolé (sol, corps du robot, bruit fugace pendant la marche) ne
# doit pas déclencher un arrêt — vérifié : un obstacle réel à 2 m donne ~75 points groupés.
OBSTACLE_MIN_POINTS = 5


class _LidarGuard:
    """Garde en mémoire la distance frontale la plus proche vue au dernier pointcloud.

    `active` : le traitement lourd n'a lieu QUE quand on marche. En cache, l'abonnement au
    LiDAR persiste entre deux appels (serveur MCP long-vécu) ; hors marche le callback doit
    rester TRIVIAL, sinon il monopolise l'exécuteur et fait échouer set_mode/set_source du
    prochain appel (cf. commentaire dans walk_distance).
    """

    def __init__(self) -> None:
        self.nearest: float | None = None  # distance du cluster proche, ou None si voie libre
        self.frames: int = 0               # nb de frames LiDAR reçues (0 => aveugle)
        self.active: bool = False

    def cb(self, msg) -> None:
        if not self.active:
            return
        from sensor_msgs_py import point_cloud2  # import différé (robot only)

        self.frames += 1
        dists = []
        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if x < ZONE_X_MIN or x > ZONE_X_MAX:
                continue
            if y < ZONE_Y_MIN or y > ZONE_Y_MAX:
                continue
            if z < ZONE_Z_MIN or z > ZONE_Z_MAX:
                continue
            dists.append(math.sqrt(x * x + y * y))

        # Anti-bruit : nearest = distance du OBSTACLE_MIN_POINTS-ème point le plus proche.
        # Il faut donc un CLUSTER d'au moins N points pour "voir" un obstacle à cette
        # distance ; un point isolé aberrant est ignoré.
        if len(dists) < OBSTACLE_MIN_POINTS:
            self.nearest = None
            return
        dists.sort()
        self.nearest = dists[OBSTACLE_MIN_POINTS - 1]


class _OdomTracker:
    """Suit la position (x, y) publiée par leg_odometry. Callback LÉGER (juste 2 floats),
    contrairement au garde LiDAR — pas de risque de monopoliser l'exécuteur.

    `traveled()` renvoie la distance parcourue depuis le dernier `reset()` (repère fixe
    leg_odom, ~157 Hz, bruit au repos < 2 mm)."""

    def __init__(self) -> None:
        self.x: float | None = None
        self.y: float | None = None
        self._x0: float | None = None
        self._y0: float | None = None

    def cb(self, msg) -> None:
        p = msg.pose.pose.position
        self.x = p.x
        self.y = p.y

    def reset(self) -> bool:
        """Fige l'origine sur la pose courante. False si aucune pose reçue encore."""
        if self.x is None:
            return False
        self._x0, self._y0 = self.x, self.y
        return True

    def traveled(self) -> float:
        if self._x0 is None or self.x is None:
            return 0.0
        return math.sqrt((self.x - self._x0) ** 2 + (self.y - self._y0) ** 2)


def walk_distance(meters: float = 1.0, speed: float = 0.25,
                  stop_distance: float = 0.8) -> str:
    """Walk the X2 humanoid forward by a given distance, stopping early if an obstacle
    is detected ahead by the chest LiDAR.

    Distance is measured in CLOSED LOOP from the robot's leg odometry (leg_odom frame): the
    robot walks at a CONSTANT speed (no ramp) until the traveled distance reaches `meters`.
    It halts immediately — before reaching the target — if any obstacle enters the frontal
    zone closer than `stop_distance`. A stop command is always published at the end.

    ⚠️ MOVES A HUMANOID. Only call with the robot standing, a clear area, and someone
    ready to trigger the emergency stop.

    Args:
        meters (float): Forward distance to travel, in meters. Defaults to 1.0. Capped at
            3.0 m per call for safety; larger values are refused (chain calls if needed).
        speed (float): Constant walking speed in m/s. Must be within [0.2, 1.0] — the
            locomotion controller rejects (and jitters on) anything in ]0, 0.2[.
            Defaults to 0.25.
        stop_distance (float): If the nearest obstacle ahead is closer than this (meters),
            the robot stops before finishing. Defaults to 0.8.

    Returns:
        str: A human-readable message: distance walked and whether it completed or was
            stopped early by an obstacle.
    """
    # Validation → on RENVOIE une chaîne (on ne lève pas) pour que l'agent se corrige.
    try:
        meters = float(meters)
        speed = float(speed)
        stop_distance = float(stop_distance)
    except (TypeError, ValueError):
        return "Invalid arguments: meters, speed and stop_distance must be numbers."

    if meters <= 0:
        return f"Invalid meters={meters}: must be a positive distance."
    if meters > MAX_DISTANCE:
        return (
            f"Refused meters={meters}: exceeds the {MAX_DISTANCE} m safety cap for a single "
            f"open-loop walk. Call again with a smaller distance (or chain several calls)."
        )
    if not (MIN_FWD_SPEED <= speed <= MAX_FWD_SPEED):
        return (
            f"Invalid speed={speed} m/s: the controller only walks at a constant speed in "
            f"[{MIN_FWD_SPEED}, {MAX_FWD_SPEED}] m/s (anything below {MIN_FWD_SPEED} makes "
            "the robot jitter instead of walking)."
        )

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    from aimdk_msgs.msg import McLocomotionVelocity  # import différé
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    vel_pub = session.get_publisher(McLocomotionVelocity, VELOCITY_TOPIC, depth=10)

    # Odométrie : callback LÉGER (2 floats), on peut l'abonner tôt sans gêner les services.
    odom = _OdomTracker()
    session.add_subscription(Odometry, ODOM_TOPIC, odom.cb, best_effort=True)

    def publish_vel(fwd: float) -> None:
        m = McLocomotionVelocity()
        m.header.stamp = session.node.get_clock().now().to_msg()
        m.source = SRC_NAME
        m.forward_velocity = float(fwd)
        m.lateral_velocity = 0.0
        m.angular_velocity = 0.0
        vel_pub.publish(m)

    # [1] mode locomotion
    # ⚠️ On s'abonne au LiDAR APRÈS set_mode/set_source. Le callback _LidarGuard.cb est
    # lourd (parcourt tout le pointcloud à 10 Hz) ; s'il est actif pendant les
    # spin_until_future_complete des services, il monopolise l'exécuteur mono-thread et
    # fait échouer set_mode/set_source (vérifié sur robot : 3/3 False avec le cb actif,
    # 3/3 True sans). L'abonnement ne démarre donc qu'au moment de marcher.
    if not session.set_mode(LOCOMOTION_MODE):
        return f"Aborted: the robot did not accept {LOCOMOTION_MODE}; it did not move."
    time.sleep(1.0)  # laisser le contrôleur stabiliser le mode

    # [2] enregistrement de la source (sinon la MC ignore les vitesses)
    if not session.set_source(INPUTACTION_ADD):
        return "Aborted: could not register the command source; the robot did not move."

    # [2b] MAINTENANT on branche le LiDAR (guard qui suit la distance frontale la plus proche).
    guard = _LidarGuard()
    session.add_subscription(PointCloud2, LIDAR_TOPIC, guard.cb, best_effort=True)

    dt = 1.0 / PUBLISH_RATE_HZ
    # Garde-fou temporel de SECOURS : si l'odométrie se fige, on ne marche pas indéfiniment.
    # On borne à 2× le temps théorique + 3 s de marge (démarrage de la démarche).
    time_cap = (meters / speed) * 2.0 + 3.0
    walked = 0.0
    blocked_at: float | None = None

    try:
        # [2c] Activer le traitement LiDAR + warm-up : 1re lecture LiDAR ET 1re pose odom
        # AVANT de bouger. Fail-safe : pas de LiDAR (aveugle) ou pas d'odométrie => refus.
        guard.active = True
        warm = 0.0
        while (guard.frames == 0 or odom.x is None) and warm < 2.0:
            session.spin_once(timeout_sec=0.1)
            warm += 0.1
        if guard.frames == 0:
            return (
                "Aborted: no LiDAR data (obstacle detection blind) — the robot did not move. "
                "Is the chest LiDAR publishing?"
            )
        if not odom.reset():
            return (
                "Aborted: no odometry data — cannot measure distance; the robot did not move. "
                "Is /aima/mc/leg_odometry publishing?"
            )
        if guard.nearest is not None and guard.nearest < stop_distance:
            return (
                f"Aborted before moving: obstacle already {guard.nearest:.2f} m ahead "
                f"(threshold {stop_distance:.2f} m)."
            )

        # [3] boucle EN BOUCLE FERMÉE : on avance tant que la distance odométrique < cible.
        start = time.monotonic()
        while True:
            session.spin_once(timeout_sec=0.0)  # traite pointcloud + odométrie
            walked = odom.traveled()
            if walked >= meters:
                break
            if time.monotonic() - start > time_cap:  # secours : odométrie figée ?
                blocked_at = None
                break
            nearest = guard.nearest
            if nearest is not None and nearest < stop_distance:
                blocked_at = nearest
                break
            publish_vel(speed)
            time.sleep(dt)
    finally:
        # [4] arrêt garanti (zéros)
        for _ in range(STOP_SETTLE_STEPS):
            publish_vel(0.0)
            session.spin_once(timeout_sec=0.0)
            time.sleep(0.05)
        # [5] libération de la source (robot laissé debout en LOCOMOTION)
        session.set_source(INPUTACTION_DELETE)
        # [6] LiDAR au repos : callback trivial pour ne pas gêner le prochain set_mode.
        guard.active = False

    walked = odom.traveled()
    if blocked_at is not None:
        return (
            f"Stopped early after {walked:.2f} m of {meters:.2f} m: obstacle detected "
            f"{blocked_at:.2f} m ahead (threshold {stop_distance:.2f} m)."
        )
    return f"Done — walked {walked:.2f} m forward (odometry-measured, target {meters:.2f} m)."
