# tools — commandes X2 packagées en tools Strands

Tools appelables par un **agent Strands tournant SUR le robot** (la carte Jetson, là où
ROS2 Humble tourne déjà). Contrairement à `../x2_ros2/` (scripts one-shot pilotés depuis le
laptop via SSH), ici l'agent appelle rclpy **directement**, dans un process long-vécu.

## Contenu

| Fichier | Rôle |
|---|---|
| `robot_session.py` | **Couche partagée** : singleton rclpy (init une fois), cache de clients, `call_service`, `set_mode`. Tous les tools passent par là. |
| `preset_motion.py` | `@tool preset_motion(motion, arm, interrupt)` — joue un geste de bras pré-programmé + `@tool list_preset_motions()` — catalogue des gestes. |
| `harness.py` | Test manuel : appelle le tool 2× dans le même process (valide le singleton). |
| `__init__.py` | `from tools import preset_motion`. |

## Le point critique : le singleton rclpy

Un agent appelle plusieurs tools dans **un seul process**. `rclpy.init()` deux fois plante,
et `rclpy.shutdown()` entre deux appels tue le contexte. `robot_session.get_session()`
initialise rclpy **une seule fois** (double-checked locking, garde `rclpy.ok()`), réutilise
le nœud et met les clients en cache. On ne fait **jamais** `shutdown()` en cours de session.

## Prérequis de lancement (IMPÉRATIF)

`import aimdk_msgs` ne résout que si l'environnement a été **sourcé AVANT** de lancer python.
Le process de l'agent doit donc démarrer depuis un shell sourcé :

```bash
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/local_setup.bash
```

Sinon `get_session()` lève `RobotUnavailableError` avec un message explicite, et les tools
renvoient une chaîne d'erreur lisible (ils ne lèvent jamais — l'agent reste dans sa boucle).

`strands-agents` doit être pip-installé dans **le même** python3.10 que ROS2 (voir
`requirements.txt`).

## Utilisation depuis un agent

```python
from strands import Agent
from tools import preset_motion, list_preset_motions

agent = Agent(tools=[preset_motion, list_preset_motions])
agent("quels gestes sais-tu faire ?")    # -> list_preset_motions()
agent("fais coucou de la main droite")   # -> preset_motion(motion="wave", arm="right")
```

Le message `aimdk_msgs/msg/McPresetMotion` définit 27 constantes, mais **vérifié sur le
robot (2026-07-07) : seule la famille 1000 (gestes de bras) est jouable** sur cette unité.
Les familles 3000 (interactions : thumbs_up, dance, pose photo…) et 4000 (tête : nod…) sont
**refusées** par le robot (`code=1, state=2`) quels que soient le mode et l'area — probablement
non provisionnées. Le catalogue `PRESETS` (dans `preset_motion.py`) ne liste donc que les
7 gestes de bras qui marchent : raise, wave, handshake, airkiss, clap, fistbump, salute.

## Vérification sur le robot

Pas de ROS2 sur le laptop → on valide sur la carte du robot.

```bash
# 1. copier le package
scp -q -r tools/ agi@172.20.10.6:/tmp/tools/

# 2. installer strands dans le python ROS2 (une fois)
ssh agi@172.20.10.6 'python3 -m pip install --user strands-agents && python3 -c "import strands; print(\"strands OK\")"'

# 3. lancer le harnais dans un shell sourcé (E-STOP EN MAIN, zone dégagée)
ssh agi@172.20.10.6 "source /opt/ros/humble/setup.bash; \
  source /home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/local_setup.bash; \
  cd /tmp && python3 -m tools.harness"
```

Succès = (a) le bras exécute les gestes, (b) les deux appels renvoient un message de succès
avec `task_id`, sans erreur « rclpy already initialized » au 2e appel.

## Note : enchaîner les gestes (`interrupt=True`)

Après un geste, le robot reste dans l'état RUNNING (« occupé ») même une fois le mouvement
physiquement terminé, et **refuse tout nouveau preset** tant qu'on envoie `interrupt=False`
(réponse `code=1, state=2`). Vérifié sur le robot : le 1er geste passe, tous les suivants
sont refusés — quel que soit le bras, quelle que soit l'attente. **Solution : `interrupt=True`**
(le défaut du tool), qui accepte le nouveau geste à tous les coups. Attendre n'y change rien ;
c'était le flag, pas la durée. (Le service `GetMcPresetMotionState` existe côté ROS mais sa
classe n'est pas dans le `aimdk_msgs` pré-compilé, donc pas de polling d'état possible en Python.)
