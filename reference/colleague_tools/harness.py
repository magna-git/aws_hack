"""Harnais de test manuel — exerce preset_motion comme le ferait l'agent Strands.

À lancer SUR le robot, dans un shell qui a sourcé ROS2 + aimdk :
    source /opt/ros/humble/setup.bash
    source /home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/local_setup.bash
    cd /tmp && python3 -m tools.harness

⚠️ Fait bouger un bras du robot. E-stop en main, zone dégagée autour des bras.

But : appeler preset_motion DEUX fois dans le même process. Si le 2e appel réussit sans
erreur « rclpy already initialized », le singleton robot_session est validé (point critique).
"""
from __future__ import annotations

# Les @tool Strands restent des fonctions Python appelables directement.
from tools.preset_motion import preset_motion


def main() -> None:
    print(">>> appel 1 : raise / right")
    print(preset_motion(motion="raise", arm="right"))

    print(">>> appel 2 (même process) : wave / left")
    print(preset_motion(motion="wave", arm="left"))

    print(">>> validation : 2 appels dans le même process sans réinit rclpy = OK singleton")


if __name__ == "__main__":
    main()
