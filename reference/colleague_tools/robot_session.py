"""Session ROS2 partagée pour les tools Strands du X2.

Un agent Strands appelle plusieurs tools DANS UN MÊME PROCESS long-vécu. Les scripts
one-shot de `x2_ros2/` font `rclpy.init()`/`rclpy.shutdown()` à chaque appel, ce qui
plante dès le 2e tool (rclpy.init() deux fois lève, et shutdown tue le contexte).

Ce module fournit un **singleton** : rclpy initialisé UNE fois, un nœud réutilisé, des
clients de service mis en cache, et trois helpers (`call_service`, `set_mode`,
`capture_frame`). Tous les futurs tools (walk, photo) passent par là.

⚠️ Contrainte d'import : AUCUN `import rclpy` / `import aimdk_msgs` au niveau module —
ils sont différés dans les fonctions. Ainsi ce fichier s'importe et se linte même sans
ROS2 (ex. sur le laptop, Python 3.14). rclpy/aimdk ne sont disponibles que sur le robot,
et uniquement si l'env ROS2 + aimdk a été sourcé AVANT de lancer le process.
"""
from __future__ import annotations

import os
import threading
from typing import Any

# Noms de services (forme URL-encodée : _5F = underscore), cf. x2_ros2/raise_arm.py.
ACTION_SRV = "/aimdk_5Fmsgs/srv/SetMcAction"
SRC_SRV = "/aimdk_5Fmsgs/srv/SetMcInputSource"
SRC_NAME = "node"

# Constantes McInputAction (vérifiées sur le robot 2026-07-07).
INPUTACTION_ADD = 1001
INPUTACTION_DELETE = 1003

# ⚠️ Profil DDS custom d'Agibot. Le shell SSH INTERACTIF l'exporte automatiquement, mais
# nos process (serveur MCP, wrappers) tournent en NON-interactif et ne l'ont pas → certains
# topics deviennent INVISIBLES (confirmé : le LiDAR /lidar_pointcloud, ce qui a cassé
# l'arrêt d'obstacle de walk_distance). On le pose avant rclpy.init() (FastRTPS le lit à la
# création du participant DDS). Cf. mémoire projet "x2-dds-profile-env".
FASTRTPS_PROFILE_PATH = "/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml"

_INIT_LOCK = threading.Lock()
_SESSION: "RobotSession | None" = None

# Dossier où les tools écrivent leurs images (sur le robot, où tourne le serveur MCP).
IMAGE_DIR = "/tmp/x2_images"


def new_image_path(prefix: str, ext: str = "jpg") -> str:
    """Chemin de fichier UNIQUE pour une image produite par un tool.

    Nom = <prefix>_<uuid4hex>.<ext> dans IMAGE_DIR (créé au besoin). L'uuid évite toute
    collision entre appels successifs ou concurrents — chaque perception a son propre fichier
    (pas d'écrasement, l'agent peut en garder plusieurs). Cf. tools perceive_front/ground.
    """
    import uuid  # import différé (léger, mais on garde le module épuré en tête)

    os.makedirs(IMAGE_DIR, exist_ok=True)
    return os.path.join(IMAGE_DIR, f"{prefix}_{uuid.uuid4().hex}.{ext}")


class RobotUnavailableError(RuntimeError):
    """Le robot / l'environnement ROS2 n'est pas joignable depuis ce process."""


class RobotSession:
    """Détient un nœud rclpy unique + un cache de clients. Ne pas instancier
    directement : passer par `get_session()`."""

    def __init__(self) -> None:
        import rclpy  # import différé

        # On DÉTIENT un nœud (pas de sous-classe) pour éviter d'importer rclpy.node.Node
        # au niveau module.
        self.node = rclpy.create_node("x2_strands_agent")
        self._clients: dict[tuple[Any, str], Any] = {}
        self._sub_cbs: dict[str, Any] = {}    # topic -> callback courant
        self._publishers: dict[tuple[Any, str], Any] = {}
        self._subs: dict[tuple[Any, str], Any] = {}
        # L'exécuteur rclpy mono-thread n'est pas sûr en spin concurrent. Strands appelle
        # les tools en séquence, mais ce verrou retire l'arête vive gratuitement.
        self._spin_lock = threading.Lock()

    def _now(self):
        return self.node.get_clock().now().to_msg()

    def get_client(self, srv_type: Any, srv_name: str):
        """Client de service mis en cache (créé au premier appel)."""
        key = (srv_type, srv_name)
        client = self._clients.get(key)
        if client is None:
            client = self.node.create_client(srv_type, srv_name)
            self._clients[key] = client
        return client

    def get_publisher(self, msg_type: Any, topic: str, depth: int = 10):
        """Publisher mis en cache (créé au premier appel)."""
        key = (msg_type, topic)
        pub = self._publishers.get(key)
        if pub is None:
            pub = self.node.create_publisher(msg_type, topic, depth)
            self._publishers[key] = pub
        return pub

    def add_subscription(self, msg_type: Any, topic: str, callback, *, best_effort: bool = True):
        """Souscription à un topic, mise en cache PAR TOPIC (comme capture_frame).

        Le callback est RÉASSIGNABLE : un 2e appel sur le même topic ne recrée pas la
        souscription mais remplace le callback courant (via `_sub_cbs`). C'est essentiel
        pour walk_distance, qui crée un nouveau garde LiDAR à chaque appel — sans ça, le
        2e appel réutiliserait le garde du 1er (distance figée = pas d'arrêt d'obstacle).
        """
        self._sub_cbs[topic] = callback
        sub = self._subs.get(topic)
        if sub is not None:
            return sub

        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        if best_effort:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            )
        else:
            qos = 10
        # Indirection : le vrai callback vit dans _sub_cbs et reste réassignable.
        sub = self.node.create_subscription(
            msg_type, topic, lambda m: self._sub_cbs[topic](m), qos)
        self._subs[topic] = sub
        return sub

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Traite les callbacks en attente (un tour d'exécuteur)."""
        import rclpy  # import différé

        with self._spin_lock:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def call_service(
        self,
        client,
        request,
        *,
        service_name: str,
        retries: int = 8,
        spin_timeout: float = 0.25,
        wait_timeout: float = 8.0,
    ):
        """Appelle un service avec le pattern de retry des scripts x2_ros2.

        Renvoie le résultat brut (chaque tool applique son propre prédicat de succès),
        ou lève RobotUnavailableError si le service ne répond pas.
        """
        import rclpy  # import différé

        with self._spin_lock:
            if not client.wait_for_service(timeout_sec=wait_timeout):
                raise RobotUnavailableError(f"service indisponible : {service_name}")
            for _ in range(retries):
                # Beaucoup de requêtes aimdk portent un header.stamp ; on le fixe si présent.
                header = getattr(request, "header", None)
                if header is not None and hasattr(header, "stamp"):
                    header.stamp = self._now()
                fut = client.call_async(request)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=spin_timeout)
                if fut.done() and fut.result() is not None:
                    return fut.result()
            raise RobotUnavailableError(f"service sans réponse après {retries} essais : {service_name}")

    def set_mode(self, mode: str) -> bool:
        """Passe le robot dans `mode` (ex. STAND_DEFAULT, LOCOMOTION_DEFAULT).

        Dédoublonne les copies identiques de x2_ros2/raise_arm.py et walk_forward.py.
        True seulement si la réponse est SUCCESS.
        """
        from aimdk_msgs.msg import CommonState  # import différé
        from aimdk_msgs.srv import SetMcAction

        client = self.get_client(SetMcAction, ACTION_SRV)
        req = SetMcAction.Request()
        req.source = SRC_NAME
        req.command.action_desc = mode
        try:
            result = self.call_service(client, req, service_name=ACTION_SRV)
        except RobotUnavailableError:
            return False
        return result.response.status.value == CommonState.SUCCESS

    def capture_frame(self, topic: str, *, timeout_s: float = 15.0) -> bytes:
        """Capture une frame depuis un topic caméra CompressedImage. Renvoie les octets
        JPEG bruts. Lève RobotUnavailableError si aucune frame en `timeout_s`.

        La caméra publie en flux : QoS BEST_EFFORT / KEEP_LAST / depth 1 = dernière frame.
        Réutilise l'abonnement par topic (mis en cache), comme get_client pour les services.
        """
        import rclpy  # import différé
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage

        holder: dict[str, bytes] = {}

        def _on_image(msg) -> None:
            if "data" not in holder:
                holder["data"] = bytes(msg.data)

        with self._spin_lock:
            sub = self._subs.get(topic)
            if sub is None:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
                # callback réassignable : on stocke la fonction courante dans un attribut
                sub = self.node.create_subscription(
                    CompressedImage, topic, lambda m: self._sub_cbs[topic](m), qos
                )
                self._subs[topic] = sub
            self._sub_cbs[topic] = _on_image

            elapsed = 0.0
            while rclpy.ok() and "data" not in holder and elapsed < timeout_s:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                elapsed += 0.5

        if "data" not in holder:
            raise RobotUnavailableError(
                f"aucune frame reçue sur {topic} en {timeout_s}s"
            )
        return holder["data"]

    def capture_pointcloud(self, topic: str, *, frames: int = 3, timeout_s: float = 15.0):
        """Capture et agrège `frames` nuages de points d'un topic PointCloud2 → tableau Nx3.

        Renvoie un np.ndarray float32 (N, 3) de points (x, y, z) dans le repère du capteur.
        Lève RobotUnavailableError si aucune frame en `timeout_s`.

        On utilise point_cloud2.read_points (pas read_points_numpy) : le pointcloud du X2 a
        des champs de types mixtes (x/y/z float + intensity/ring autre type), que la variante
        numpy refuse. QoS BEST_EFFORT comme le flux LiDAR.
        """
        import numpy as np  # import différé (robot)
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        chunks: list = []
        state = {"seen": 0}

        def _on_cloud(msg) -> None:
            if state["seen"] >= frames:
                return
            arr = np.array(
                [[x, y, z] for x, y, z in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True)],
                dtype=np.float32,
            ).reshape(-1, 3)
            chunks.append(arr)
            state["seen"] += 1

        with self._spin_lock:
            sub = self._subs.get(topic)
            if sub is None:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                )
                sub = self.node.create_subscription(
                    PointCloud2, topic, lambda m: self._sub_cbs[topic](m), qos)
                self._subs[topic] = sub
            self._sub_cbs[topic] = _on_cloud

            elapsed = 0.0
            while rclpy.ok() and state["seen"] < frames and elapsed < timeout_s:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                elapsed += 0.5

        if not chunks:
            raise RobotUnavailableError(f"aucun nuage reçu sur {topic} en {timeout_s}s")
        return np.vstack(chunks)

    def capture_depth(self, topic: str, *, timeout_s: float = 15.0):
        """Capture une image de profondeur (sensor_msgs/Image, encodage 16UC1 en mm) → mètres.

        Renvoie un np.ndarray float32 HxW en mètres (0.0 = pas de mesure). Lève
        RobotUnavailableError si aucune frame en `timeout_s`.
        """
        import numpy as np  # import différé (robot)
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        holder: dict = {}

        def _on_depth(msg) -> None:
            if "d" in holder:
                return
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
            holder["d"] = raw.astype(np.float32) * 0.001  # mm -> m

        with self._spin_lock:
            sub = self._subs.get(topic)
            if sub is None:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
                sub = self.node.create_subscription(
                    Image, topic, lambda m: self._sub_cbs[topic](m), qos)
                self._subs[topic] = sub
            self._sub_cbs[topic] = _on_depth

            elapsed = 0.0
            while rclpy.ok() and "d" not in holder and elapsed < timeout_s:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                elapsed += 0.5

        if "d" not in holder:
            raise RobotUnavailableError(f"aucune image depth reçue sur {topic} en {timeout_s}s")
        return holder["d"]

    def set_source(self, action_value: int, *, priority: int = 40, timeout: int = 1000) -> bool:
        """Enregistre/désenregistre une source de commande MC (ADD=1001 / DELETE=1003).

        Sans source enregistrée, la MC ignore les vitesses publiées. Extrait de
        x2_ros2/walk_forward.py. True dès que le service répond (l'état est logué à part).
        """
        from aimdk_msgs.srv import SetMcInputSource  # import différé

        client = self.get_client(SetMcInputSource, SRC_SRV)
        req = SetMcInputSource.Request()
        req.action.value = action_value
        req.input_source.name = SRC_NAME
        req.input_source.priority = priority
        req.input_source.timeout = timeout
        try:
            self.call_service(client, req, service_name=SRC_SRV)
        except RobotUnavailableError:
            return False
        return True


def get_session() -> RobotSession:
    """Accesseur singleton. Initialise rclpy une seule fois, réutilise le nœud.

    Lève RobotUnavailableError si ROS2/aimdk ne sont pas importables (env non sourcé).
    Ne fait JAMAIS rclpy.shutdown() (le contexte vit tout le process).
    """
    global _SESSION
    if _SESSION is not None:
        return _SESSION
    with _INIT_LOCK:
        if _SESSION is None:
            try:
                import rclpy
            except ImportError as exc:
                raise RobotUnavailableError(
                    "rclpy introuvable — ROS2 non sourcé avant le lancement du process ?"
                ) from exc
            try:
                import aimdk_msgs  # noqa: F401 — vérifie juste la dispo
            except ImportError as exc:
                raise RobotUnavailableError(
                    "aimdk_msgs introuvable — as-tu sourcé "
                    "/home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/"
                    "local_setup.bash avant de lancer l'agent ?"
                ) from exc
            # Poser le profil DDS AVANT rclpy.init() si absent (shell non-interactif) et si
            # le fichier existe. Sans lui, certains topics (LiDAR) sont invisibles.
            if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") and os.path.exists(
                FASTRTPS_PROFILE_PATH
            ):
                os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = FASTRTPS_PROFILE_PATH
            if not rclpy.ok():
                rclpy.init()
            _SESSION = RobotSession()
    return _SESSION
