"""Tool Strands : synthèse vocale (TTS) du X2.

`speak(text, interrupt, priority)` fait prononcer un texte au robot via le service
`/aimdk_5Fmsgs/srv/PlayTts` (`aimdk_msgs/srv/PlayTts`).

⚠️ LANGUE : validé en direct 2026-07-07, le TTS ne rend correctement qu'en **ANGLAIS**.
Le français/chinois sont acceptés par le service (is_success=True) mais mal prononcés →
donner à l'agent des textes anglais.

Le service renvoie toujours `is_success=True` dès que le texte est accepté (et
`estimated_duration` n'est pas renseigné) ; on se fie donc à `is_success`. Purement audio,
aucun mouvement moteur → sûr.

Fichier PUR sans décorateur : les couches Strands/MCP l'enveloppent depuis le registre.
rclpy/aimdk sont derrière robot_session, donc ce fichier s'importe même sans ROS2.
"""
from __future__ import annotations

from tools.robot_session import RobotUnavailableError, get_session

# Service TTS (forme URL-encodée : _5F = underscore).
TTS_SRV = "/aimdk_5Fmsgs/srv/PlayTts"

# Niveaux de priorité TtsPriorityLevel (cf. définition PlayTts sur le robot).
# INTERACTION_L6 = usage normal "le robot s'adresse à quelqu'un".
PRIORITY_INTERACTION = 6

# Garde-fou : le TTS bloque un temps proportionnel au texte ; on borne pour la démo.
MAX_TEXT_LEN = 500


def speak(text: str, interrupt: bool = True, priority: int = PRIORITY_INTERACTION) -> str:
    """Make the X2 robot speak a sentence out loud using text-to-speech.

    Plays synthesized speech through the robot's speakers. Audio only — no motor movement,
    always safe to call.

    IMPORTANT: the robot's TTS only sounds correct in ENGLISH. Pass English text; other
    languages are accepted but mispronounced.

    Args:
        text (str): What the robot should say, in English. Keep it to a sentence or two.
        interrupt (bool): Interrupt any speech currently playing and say this instead.
            Defaults to True.
        priority (int): TTS priority level (1=background ... 6=interaction ... 10=safety).
            Defaults to 6 (normal interaction).

    Returns:
        str: A human-readable success or failure message describing what happened.
    """
    said = str(text).strip()
    if not said:
        return "Nothing to say: `text` was empty."
    if len(said) > MAX_TEXT_LEN:
        return (
            f"Text too long ({len(said)} chars, max {MAX_TEXT_LEN}). "
            "Split it into shorter sentences."
        )

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    from aimdk_msgs.srv import PlayTts  # import différé (dispo seulement sur le robot)

    client = session.get_client(PlayTts, TTS_SRV)
    req = PlayTts.Request()
    req.tts_req.text = said
    req.tts_req.priority_level.value = int(priority)
    req.tts_req.is_interrupted = bool(interrupt)

    try:
        result = session.call_service(client, req, service_name=TTS_SRV)
    except RobotUnavailableError:
        return (
            f"The robot's TTS service ({TTS_SRV}) did not respond; nothing was said."
        )

    resp = getattr(result, "tts_resp", None)
    if resp is not None and getattr(resp, "is_success", False):
        return f"Done — the robot said: {said!r}."
    err = getattr(resp, "error_message", "") if resp is not None else ""
    return f"The robot rejected the speech request (error={err!r})."
