# AgiBot X2 — inventaire des capacités (robot `agi@172.20.10.6`)

Investigation read-only faite le 2026-07-08, en complément de `SESSION_SUMMARY.md`.
Aucune commande envoyée au robot, aucune installation, aucun script de `/tmp/tools/` exécuté.

## 🚨 Découverte la plus importante de cette investigation

**Le "LiDAR intermittent" qui a posé problème toute la session n'est probablement pas un problème matériel.**

Un fichier de profil DDS custom existe : `/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml`
(transport SHM + UDPv4 whitelisté). La variable d'environnement `FASTRTPS_DEFAULT_PROFILES_FILE`
doit pointer dessus **avant** `rclpy.init()`, sinon certains topics (confirmé : le LiDAR) sont
invisibles pour ce process — documenté par le collègue lui-même dans `tools/robot_session.py`
(commentaire + fix automatique dans `get_session()`), suite à un problème qu'il avait déjà eu avec
`walk_distance`.

**Testé en A/B pendant cette investigation :**
```bash
# SANS la variable :
$ ros2 topic hz /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
WARNING: topic [...] does not appear to be published yet

# AVEC la variable :
$ export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml
$ ros2 topic hz /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
average rate: 10.081
average rate: 10.043
```

**Action recommandée** : dans toute commande/script non-interactif lancé sur le robot (y compris
depuis `x2_safety_toolkit`), exporter cette variable avant de sourcer ROS2 :
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml
source /opt/ros/humble/setup.bash
```
Ceci explique très probablement une bonne partie des échecs "le LiDAR ne répond pas" rencontrés
via SSH non-interactif pendant la session (nos commandes `ssh agi@... "..."` n'ont jamais posé
cette variable). Cela ne veut pas dire que le capteur n'a JAMAIS été réellement inactif (le robot
a aussi pu changer d'état physique), mais ce facteur logiciel est confirmé et suffisant à lui seul
pour reproduire le symptôme.

## 🚨 Deuxième découverte importante : sudo n'est pas aussi bloqué qu'on le pensait

`sudo -n -l` (sans mot de passe) montre :
```
User agi may run the following commands on agi:
    (root) NOPASSWD: /usr/sbin/agibot_reset_user_rootfs.sh
    (root) NOPASSWD: /usr/bin/apt, /usr/bin/nmcli, /usr/bin/nmtui, /usr/sbin/ifconfig
```
On avait testé `sudo apt-get install ...` (refusé). **On n'a jamais testé `sudo apt install ...`**
(sans le `-get`) — hors, c'est précisément `/usr/bin/apt` (pas `/usr/bin/apt-get`) qui est
whitelisté en NOPASSWD. Ça pourrait débloquer l'installation de `ros-humble-slam-toolbox` sans
avoir besoin du mot de passe root ni du support AgiBot. **Non testé ici** (hors scope de cette
investigation read-only) — à essayer : `sudo apt install ros-humble-slam-toolbox
ros-humble-pointcloud-to-laserscan`.

---

## ✅ CONFIRMÉ POSSIBLE (testé avec de vraies données, cette session ou cette investigation)

- **LiDAR poitrine** (`lidar_chest_front`) : PointCloud2 réel, ~10-11 Hz, ~25 000 points/scan,
  champs `x,y,z,intensity,ring,timestamp`. Fonctionne de façon fiable **si**
  `FASTRTPS_DEFAULT_PROFILES_FILE` est posé (voir ci-dessus).
- **Odométrie des jambes** (`/aima/mc/leg_odometry`, `nav_msgs/Odometry`) : ~130-150 Hz, position+
  orientation de `lidar_imu_chest_front` dans le repère `leg_odom`. QoS `BEST_EFFORT`.
- **`/tf`** : arbre complet des transforms articulaires (bras, jambes, tête, taille) en direct
  quand le robot est actif.
- **Notre `x2_safety_toolkit`** (7 nodes ROS2 Python) : inspection LiDAR, détection d'obstacles,
  carte locale 4×4m, carte persistante 20×20m via odométrie ("SLAM maison"), contrôleur de
  vitesse dry-run-first, visu ASCII, snapshot PGM+YAML. Détails complets dans `SESSION_SUMMARY.md`.
- **`walk_distance`** (tool du collègue) : marche avant avec détection d'obstacle robuste
  (cluster ≥5 points, arrêt avant/pendant, distance en boucle fermée via odométrie), vérifié en
  direct par l'utilisateur (le robot a bougé, forward=0.2 m/s en état CLEAR).
- **`obstacle_avoidance_node`** (SDK officiel `py_examples`) : lancé et vérifié — publie
  forward/stop+turn sur `/aima/mc/locomotion/velocity` selon la zone frontale LiDAR.
- **`turn_in_place`** (tool du collègue) : rotation par pulses calibrées empiriquement
  (45°/18°), **mais sans aucune détection d'obstacle** (confirmé en lisant le code).
- **`preset_motion`** : seule la famille de gestes de bras ("1000" : raise, wave, handshake,
  airkiss, clap, fistbump, salute) fonctionne sur cette unité ; familles 3000 (interactions)
  et 4000 (tête) refusées par le robot (`code=1, state=2`), vérifié empiriquement par le collègue.
- **TTS (`speak`)** : fonctionne, mais **seulement en anglais** — le français/chinois sont
  acceptés (`is_success=True`) mais mal prononcés (vérifié empiriquement).
- **Expressions faciales (`play_emotion`)** : blink/close/open/happy vérifiés en direct par le
  collègue ; catalogue de 26 expressions au total, purement affichage écran, aucun mouvement.
- **`capture_photo`** : caméra tête avant, JPEG 2688×1944, publie spontanément à 30 Hz (vérifié).
- **`perceive_front`** : fusion LiDAR + caméra avant avec calibration extrinsèque figée
  (translation 26.7 cm poitrine↔tête), vérifiée sur le robot.
- **`perceive_ground`** : caméra RGB-D tête (Orbbec Gemini335), couleur+profondeur alignées
  pixel-à-pixel, portée ~3.7 m, montée inclinée ~50° (image retournée 180° dans le code).
- **Strands agent framework** : `strands-agents==1.45.0` installé et importable dans le python
  ROS2 du robot. Un node `/x2_strands_agent` est visible dans le graphe ROS2 (le collègue a donc
  déjà fait tourner son agent sur cette machine).
- **Serveur MCP** (`x2_mcp/server.py`, créé/modifié le jour même de cette investigation) : expose
  les mêmes tools via le protocole MCP (stdio), avec un piège documenté et déjà corrigé (le SDK
  MCP ne transmet pas `PYTHONPATH`/`AMENT_PREFIX_PATH` par défaut à un serveur lancé en
  sous-processus — fix : passer `env=dict(os.environ)` explicitement).

## 🟡 DISPONIBLE MAIS NON TESTÉ (existe dans le graphe ROS2, jamais vérifié empiriquement)

Volume important de **services "normaux"** (types `aimdk_msgs/srv/*`, pas de wrapper protobuf —
donc appelables directement via `ros2 service call` ou un client rclpy standard) :

- **Mains** : `SetHandsActions`, `GetHandType` — contrôle des mains/doigts, jamais testé.
- **LED/lumières** : `SetPmuLed`, `PlayLight`, `LedStripCommand`, `ShowRGB`.
- **Sécurité** : `TriggerEStop`, `ReleaseEStop`, `ActivateMcSecureAction`, `SetMcSecureMotion` —
  un arrêt d'urgence logiciel existe et est appelable en service ROS2, jamais testé par nous.
- **Audio** : `PlayAudioFile`, `PlayMediaFile`, `RequestAudioFocus`, `SetVolume`/`GetVolume`,
  `SetMute`/`GetMute`, capture micro (`CaptureAudioFile`).
- **Vidéo/émoji supplémentaires** : `PlayVideo`, `PlayVideoGroup`, `PlayEmojiGroup`,
  `PlayDefaultEmoji`, `PlayAudioEmotionMotionByPath`.
- **État robot / diagnostics** : `GetAllJointState`, `GetAllJointIsReady`, `GetRobotState`,
  `GetSystemState`, `GetHdsAlertInfo`, `GetHdsDiagnosticInfo`, `GetRobotInfo`.
- **Réseau** : `ConnectWifiHalow`, `CtrlWifi`, `CtrlFiveG`, `ConnectBluetooth` — le robot a du
  WiFi HaLow et de la 5G en plus du WiFi classique, jamais exploré.
- **Caméra tête (Orbbec)** : ~35 services de configuration (exposition, gain, balance des blancs,
  rotation, filtre, LDP, laser…) — contrôle fin jamais utilisé.
- **`GetMcPresetMotionState`** : le collègue note que la *classe* de ce service n'est pas dans
  l'`aimdk_msgs` précompilé disponible → pas de polling d'état de geste possible en Python
  actuellement, malgré le service existant côté ROS.
- **Packages nav2 partiellement installés** : `nav2_costmap_2d`, `nav2_common`, `nav2_util`,
  `nav2_msgs`, `nav2_voxel_grid`, `grid_map_costmap_2d` sont présents (mais PAS
  `nav2_map_server` ni `slam_toolbox` — voir section bloquée). Une brique de costmap existe
  donc déjà partiellement ; non explorée.
- **Topics de perception "officiels" mais silencieux au moment du test** : `/perception/grid_map`,
  `/perception/tracked_obstacles`, `/costmap_update_map`, `/dbg_static_map`, `/mono_perception/*`
  — Publisher count = 0 au moment du test (le stack de perception/nav n'était simplement pas
  démarré, pas forcément inaccessible).

## 🔴 BLOQUÉ / INACCESSIBLE (avec la raison précise)

- **SLAM/mapping officiel** (`MappingService` : `StartMapping`, `GetStoredMapNames`,
  `Get2DWholeMap`, etc.) : type `ros2_plugin_proto/srv/RosRpcWrapper` — wrapper protobuf
  générique propriétaire, schéma non documenté dans le SDK public. Inutilisable via
  `ros2 service call` classique. Idem pour `LocalizationService`, `RelocalizationService`,
  `EmAppService`, `HalBmsService`, `TaskEngineInterfaceService`, `SmNodeService` (une par
  sous-système : `hal_sensor_orin`, `mc`, `map_manager`, `slam`, `perception`, `pnc`, etc.),
  `CloudPropertyService`, `RecordPlaybackService` — **~15 familles de services**, toutes
  RosRpcWrapper, donc toutes dans la même impasse CLI.
- **Introspection de certains messages custom via `ros2 topic echo`** : `aimdk_msgs/msg/ModuleInfo`
  et `aimdk_msgs/msg/SmSystemState` (entre autres) donnent "message type invalid" même avec
  l'environnement sourcé — la définition Python de ces messages n'est pas dans les bindings
  `aimdk_msgs` disponibles sur cette unité. Limite du SDK, pas du robot : le topic existe et
  publie, juste illisible en CLI sans écrire un client Python dédié.
- **`sudo apt-get install`** : refusé explicitement (whitelist sudoers ne liste que
  `/usr/bin/apt`, pas `/usr/bin/apt-get` — voir découverte en haut de ce document, à re-tester
  avec `apt` directement).
- **PC de développement (ROS2 Jazzy, ce poste) ne voit aucun topic du robot** : `ros2 topic list`
  ne renvoie que `/parameter_events` et `/rosout` même connecté au même sous-réseau
  (`172.20.10.0/28`, hotspot mobile). Cause probable : le hotspot bloque le trafic
  multicast/broadcast entre appareils connectés (comportement standard des hotspots iPhone).
  Testé aussi avec Docker `osrf/ros:humble-desktop` en mode réseau `host` (même distro que le
  robot) : toujours rien — donc pas un problème de version ROS, bien un problème réseau/multicast.
  **Conséquence structurelle** : toute interaction doit passer par SSH direct vers
  `agi@172.20.10.6`, jamais par découverte DDS directe depuis ce PC.
- **Boot USB pour contourner sudo** : abandonné — techniquement non viable (le robot est un
  Jetson Orin ARM avec sa propre procédure de flash NVIDIA, pas un boot USB générique x86) et
  risqué (robot potentiellement actif/debout physiquement).
