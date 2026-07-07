"""Registre explicite des tools du robot.

Source de vérité unique de la liste des fonctions-tool. Les couches d'exposition
(tools/strands_tools.py pour Strands, x2_mcp/server.py pour MCP) itèrent dessus.

Ajouter un tool = importer sa fonction pure et l'ajouter à TOOLS. Ni la couche Strands
ni le serveur MCP n'ont alors besoin d'être modifiés.
"""
from tools.capture_photo import capture_photo
from tools.perceive_front import perceive_front
from tools.perceive_ground import perceive_ground
from tools.play_emotion import list_emotions, play_emotion
from tools.preset_motion import list_preset_motions, preset_motion
from tools.speak import speak
from tools.turn_in_place import turn_in_place
from tools.upload_photo_s3 import capture_and_upload_photo
from tools.walk_distance import walk_distance

# Fonctions PURES (sans décorateur). L'ordre n'a pas d'importance.
TOOLS = [preset_motion, list_preset_motions, capture_photo, walk_distance,
         play_emotion, list_emotions, speak, turn_in_place,
         perceive_front, perceive_ground, capture_and_upload_photo]
