# Résumé de session — AgiBot X2, perception & agent Strands

Date : 2026-07-07/08
Robot : AgiBot X2 (humanoïde), unité de calcul dev `agi@172.20.10.6` (Jetson Orin, JetPack 6 / L4T r36.4, ROS2 Humble, aarch64)
Poste dev : ce PC (ROS2 Jazzy), connecté au robot via un hotspot mobile (réseau `172.20.10.0/28`)

## Contexte de départ

Objectif : le SLAM officiel d'AgiBot X2 n'est pas accessible facilement (services protobuf propriétaires, cf. plus bas), donc construire une brique de perception/sécurité alternative basée sur le LiDAR poitrine (`lidar_chest_front`).

## Ce qu'on a construit : `x2_safety_toolkit`

Package ROS2 Python créé dans `~/hackathon/aimdk_robot/src/x2_safety_toolkit/` (local ET déployé sur le robot dans `~/aimdk/src/x2_safety_toolkit/`), buildé avec `colcon build --packages-select x2_safety_toolkit`.

### Les 7 nodes

| Node | Rôle | S'abonne à | Publie |
|---|---|---|---|
| `lidar_inspector_node` | Diagnostic capteur (fps, frame_id, nb points) | `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | *(rien)* |
| `obstacle_detector_node` | Distance min avant + dégagement gauche/droite | LiDAR | `/x2/obstacle_status` (JSON), `/x2/debug_points` |
| `local_safety_map_node` | Grille locale instantanée 4×4m, se redessine à chaque scan | LiDAR | `/x2/local_safety_map` |
| `global_map_builder_node` | Carte persistante 20×20m, accumulée dans le temps via l'odométrie des jambes ("SLAM maison", sans loop closure) | LiDAR + `/aima/mc/leg_odometry` | `/x2/global_map` |
| `safe_velocity_controller_node` | Décide stop/slow/forward à partir de `/x2/obstacle_status`. `dry_run=true` par défaut : ne publie **jamais** de commande sauf si explicitement `dry_run:=false` | `/x2/obstacle_status` | `/aima/mc/locomotion/velocity` *(si dry_run=false)* |
| `map_ascii_view` | Visualisation texte de la carte, direct dans le terminal SSH | `/x2/local_safety_map` | *(rien)* |
| `map_snapshot_saver` | Sauvegarde un instantané de la carte en PGM+YAML (format standard ROS/nav2), un seul message puis s'arrête | `/x2/local_safety_map` | *(rien, écrit un fichier)* |

Plus un **launch file** (`launch/x2_safety_toolkit.launch.py`) : liste de nodes à commenter/décommenter directement dans le fichier (pas d'arguments CLI), pour choisir quoi lancer sans taper de longues commandes.

### Décisions techniques notables

- **Sécurité par défaut** : aucun node ne publie de commande de mouvement sauf `safe_velocity_controller_node`, et seulement avec `dry_run:=false` explicite.
- **QoS** : tous les abonnements LiDAR/odométrie utilisent `BEST_EFFORT` (les publishers du robot sont `RELIABLE`/`BEST_EFFORT` + `TRANSIENT_LOCAL` — compatible). Un oubli de ce point a fait planter `global_map_builder_node` au début (`leg_odometry` nécessite `BEST_EFFORT`, pas le défaut `RELIABLE`).
- **Bug de précision flottante corrigé** : l'indexation de cellule de grille (`int(valeur/résolution)`) pouvait basculer d'une cellule à cause du bruit flottant pile sur les frontières. Corrigé avec un epsilon centralisé (`to_cell_index()` dans `common.py`).
- **Rotation quaternion** : implémentée à la main (pas de dépendance `tf_transformations`), vérifiée numériquement (rotation 90° testée).

### Tests effectués

- Logique validée en local avec données simulées (obstacle_detector, local_map, global_map : accumulation, persistance après déplacement, rotation).
- Déployé et testé en conditions réelles sur le robot : LiDAR confirmé actif par intermittence (~10-11Hz, ~25 000 points/scan quand actif), carte locale confirmée avec vraies données occupé/libre.
- Un instantané PGM réel a été récupéré, converti et affiché à l'utilisateur.

## Problème rencontré : LiDAR intermittent

Le topic `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` s'est arrêté de publier plusieurs fois pendant la session (confirmé indépendamment via `ros2 topic hz`/`echo`, donc pas un bug de notre code). Semble lié à l'état du robot (debout/actif vs inactif). **Point de vigilance pour la suite** : tout ce qui dépend du LiDAR (nos nodes + les tools `perceive_front`/`walk_distance` du collègue) peut sembler "cassé" alors que c'est le capteur qui est en veille.

## Piste explorée : SLAM officiel

- Un vrai stack de SLAM/mapping **existe et tourne** sur le robot (`soc1_map_manager`, topics `/map`, `/slam/lidar_odom`, service `MappingService` avec `StartMapping`/`GetStoredMapNames`/etc.).
- Inaccessible simplement : ces services utilisent un wrapper protobuf générique (`ros2_plugin_proto/srv/RosRpcWrapper`), schéma propriétaire non documenté dans le SDK public.
- Tentative d'installer `slam_toolbox` (open-source) via `apt` : bloquée, l'utilisateur `agi` a un `sudo` restreint (whitelist de commandes, pas d'installation de paquets).
- Abandon de la piste "boot USB pour bypasser sudo" : techniquement non-viable (Jetson ARM, pas de boot USB générique x86) et risqué (robot potentiellement actif/debout).
- **Solution retenue** : `global_map_builder_node` (voir ci-dessus) — mapping approximatif sans loop closure, en réutilisant l'odométrie déjà fournie par le robot (`/aima/mc/leg_odometry`), sans rien installer.

## Découverte : le projet du collègue (`/tmp/tools/` sur le robot)

Framework d'agent LLM **Strands** tournant directement sur la carte du robot, avec des tools Python purs (sans décorateur, enveloppés en `@tool` via `strands_tools.py`) :

| Tool | Rôle |
|---|---|
| `perceive_front` | Photo caméra avant + nuage LiDAR projeté dessus (calibration figée), coloré par distance |
| `perceive_ground` | Caméra RGB-D tête (couleur+profondeur alignées), vue sol/proche |
| `preset_motion` / `list_preset_motions` | Gestes de bras pré-programmés (seule la famille "1000" — bras — fonctionne sur cette unité ; familles 3000/4000 refusées) |
| `walk_distance` | Marche avant avec **détection d'obstacle solide** : zone frontale identique à la nôtre, anti-bruit par cluster (≥5 points), arrêt avant/pendant la marche, distance mesurée en boucle fermée via odométrie, plafond 3m/vitesse [0.2,1.0] |
| `turn_in_place` | Rotation par crans calibrés (45°/18°) — **aucune détection d'obstacle** (limite reconnue : le LiDAR ne voit que devant, protection partielle possible seulement) |
| `speak`, `capture_photo`, `play_emotion`, `upload_photo_s3`, `probe` | Non explorés en détail |
| `robot_session.py` | Singleton rclpy partagé entre tous les tools (init une seule fois par process agent) |
| `registry.py` | Liste centrale des tools purs |

**Conclusion sur l'idée `check_obstacles`** : abandonnée — `walk_distance` a déjà une détection robuste, donc redondante. Le vrai écart identifié est l'absence totale de détection dans `turn_in_place`, mais la portée d'une amélioration y est limitée par la couverture avant-seulement du LiDAR.

## Pistes pour la suite (non faites, à décider)

- Ajouter une vérification pré-rotation basique à `turn_in_place` (portée limitée, voir ci-dessus).
- Prévenir le collègue du problème LiDAR intermittent (impacte directement `perceive_front`/`walk_distance`).
- Éventuellement demander l'accès admin/support AgiBot pour débloquer soit le SLAM officiel soit l'installation de `slam_toolbox`.
- Voir `ROBOT_CAPABILITIES.md` (en cours de génération) pour l'inventaire complet de ce qui est possible/bloqué sur ce robot.
