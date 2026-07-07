"""Tools pour piloter le X2 depuis le robot.

- Fonctions pures : `from tools import preset_motion, list_preset_motions, capture_photo`
- Registre : `from tools.registry import TOOLS`
- Couche Strands (nécessite strands) : `from tools.strands_tools import STRANDS_TOOLS`
- Serveur MCP : voir `x2_mcp/server.py`
"""
from tools.capture_photo import capture_photo
from tools.preset_motion import list_preset_motions, preset_motion
from tools.registry import TOOLS

__all__ = ["preset_motion", "list_preset_motions", "capture_photo", "TOOLS"]
