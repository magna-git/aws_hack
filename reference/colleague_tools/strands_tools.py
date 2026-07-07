"""Couche d'exposition Strands : enveloppe les fonctions pures du registre en @tool.

À importer par un agent Strands tournant SUR le robot :
    from tools.strands_tools import STRANDS_TOOLS
    agent = Agent(tools=STRANDS_TOOLS)

`strands` est une vraie dépendance (pip) présente sur le robot ; ce module n'est donc
importable que là où strands est installé (pas sur le laptop nu).
"""
from strands import tool

from tools.registry import TOOLS

# Chaque fonction pure enveloppée en tool Strands. tool(fn) lit signature + docstring.
STRANDS_TOOLS = [tool(fn) for fn in TOOLS]
