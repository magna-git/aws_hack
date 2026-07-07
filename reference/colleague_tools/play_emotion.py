"""Tools Strands : expressions faciales / yeux du X2.

- `list_emotions()` : catalogue des expressions jouables (pour que l'agent découvre les
  noms au lieu de deviner).
- `play_emotion(emotion, loop, priority)` : joue une expression via le service
  `/face_ui_proxy/play_emoji` (`aimdk_msgs/srv/PlayEmoji`).

Purement de l'affichage sur le visage/les yeux — AUCUN mouvement moteur, donc sûr. Les ids
proviennent de l'énumération du message PlayEmoji, vérifiée sur le robot (2026-07-07) ;
cligner (1) / fermer (40) / ouvrir (50) / content (90) ont été joués en direct → success.

Fichier PUR sans décorateur : les couches Strands/MCP l'enveloppent depuis le registre.
rclpy/aimdk sont derrière robot_session, donc ce fichier s'importe même sans ROS2.
"""
from __future__ import annotations

from tools.robot_session import RobotUnavailableError, get_session

# Service piloté par le proxy de l'écran facial.
EMOJI_SRV = "/face_ui_proxy/play_emoji"

MODE_ONCE = 1
MODE_LOOP = 2

# Catalogue : nom convivial -> (emotion_id, description).
# Ids issus de l'enum aimdk_msgs/srv/PlayEmoji. `idle=True` = animation de repos, plus
# naturelle en boucle ; les autres sont des expressions ponctuelles (mode ONCE par défaut).
EMOTIONS: dict[str, dict] = {
    # Idle / repos (animations douces)
    "blink":       {"id": 1,   "idle": True,  "desc": "blink the eyes"},
    "calm":        {"id": 10,  "idle": True,  "desc": "calm resting eyes"},
    "calm2":       {"id": 11,  "idle": True,  "desc": "calm resting eyes (variant)"},
    "game":        {"id": 20,  "idle": True,  "desc": "playful/game idle"},
    "cute":        {"id": 30,  "idle": True,  "desc": "cute idle"},
    "cute2":       {"id": 31,  "idle": True,  "desc": "cute idle (variant)"},
    "cute3":       {"id": 32,  "idle": True,  "desc": "cute idle (variant)"},
    "cute4":       {"id": 33,  "idle": True,  "desc": "cute idle (variant)"},
    # États des yeux
    "close":       {"id": 40,  "idle": False, "desc": "close the eyes"},
    "open":        {"id": 50,  "idle": False, "desc": "open the eyes"},
    "bored":       {"id": 60,  "idle": False, "desc": "bored look"},
    "abnormal":    {"id": 70,  "idle": False, "desc": "error/abnormal look"},
    "sleepy":      {"id": 80,  "idle": False, "desc": "sleepy / falling asleep"},
    "charge":      {"id": 220, "idle": False, "desc": "charging eyes"},
    # Émotions
    "happy":       {"id": 90,  "idle": False, "desc": "happy"},
    "veryhappy":   {"id": 100, "idle": False, "desc": "very happy"},
    "ecstatic":    {"id": 101, "idle": False, "desc": "ecstatic / overjoyed"},
    "sad":         {"id": 110, "idle": False, "desc": "sad"},
    "sympathy":    {"id": 120, "idle": False, "desc": "sympathy / compassion"},
    "confused":    {"id": 130, "idle": False, "desc": "confused / puzzled"},
    "shocked":     {"id": 140, "idle": False, "desc": "shocked / surprised"},
    "actcute":     {"id": 150, "idle": False, "desc": "acting cute / coy"},
    "serious":     {"id": 160, "idle": False, "desc": "serious"},
    "thinking":    {"id": 170, "idle": False, "desc": "thinking"},
    "angry":       {"id": 180, "idle": False, "desc": "angry"},
    "veryangry":   {"id": 190, "idle": False, "desc": "very angry / furious"},
    "adore":       {"id": 200, "idle": False, "desc": "adoring"},
    "veryadore":   {"id": 210, "idle": False, "desc": "deeply adoring"},
}

# Alias tolérants : formulations libres du LLM -> clé du catalogue.
_EMOTION_ALIASES = {
    "wink": "blink", "blink eyes": "blink",
    "close eyes": "close", "shut eyes": "close", "eyes closed": "close",
    "open eyes": "open", "eyes open": "open", "wake": "open", "wake up": "open",
    "neutral": "calm", "idle": "calm", "rest": "calm", "resting": "calm",
    "joy": "happy", "smile": "happy", "glad": "happy",
    "very happy": "veryhappy", "overjoyed": "ecstatic", "excited": "ecstatic",
    "unhappy": "sad", "sorrow": "sad",
    "confuse": "confused", "puzzled": "confused",
    "shock": "shocked", "surprised": "shocked", "surprise": "shocked",
    "think": "thinking", "pondering": "thinking",
    "mad": "angry", "furious": "veryangry", "very angry": "veryangry",
    "sleep": "sleepy", "tired": "sleepy",
    "boring": "bored", "error": "abnormal", "charging": "charge",
}


def list_emotions() -> str:
    """List the facial expressions the X2 can display on its eyes/face.

    Use this to discover the available expression names before calling `play_emotion`.
    These are display-only (eyes and face) and involve no motor movement.

    Returns:
        str: A readable list of expression names grouped by kind.
    """
    idle = [f"  - {n}: {m['desc']}" for n, m in EMOTIONS.items() if m["idle"]]
    other = [f"  - {n}: {m['desc']}" for n, m in EMOTIONS.items() if not m["idle"]]
    return "\n".join(
        [
            f"The X2 can display {len(EMOTIONS)} facial expressions on its eyes. "
            "Call play_emotion(emotion=<name>, loop=<true|false>).",
            "",
            "Idle / resting animations (natural to loop):",
            *idle,
            "",
            "Eye states & emotions (usually played once):",
            *other,
        ]
    )


def play_emotion(emotion: str, loop: bool = False, priority: int = 10) -> str:
    """Make the X2 display a facial expression on its eyes (blink, happy, sad, thinking...).

    Drives the robot's face/eye screen only — no motor movement, always safe to call. Call
    `list_emotions` first to see all available expression names.

    Args:
        emotion (str): Expression name from the catalogue (e.g. "blink", "happy", "sad",
            "thinking", "close", "open"). See `list_emotions` for the full list.
        loop (bool): Play the animation in a loop instead of once. Defaults to False.
            Idle expressions (blink, calm, cute) look natural looped; emotions are usually
            played once.
        priority (int): Playback priority; higher wins over concurrent requests. Defaults 10.

    Returns:
        str: A human-readable success or failure message describing what happened.
    """
    emotion_key = str(emotion).strip().lower()
    emotion_key = _EMOTION_ALIASES.get(emotion_key, emotion_key)

    if emotion_key not in EMOTIONS:
        return (
            f"Invalid emotion '{emotion}'. Call list_emotions to see the available "
            f"expression names."
        )

    meta = EMOTIONS[emotion_key]
    emotion_id = meta["id"]
    mode = MODE_LOOP if loop else MODE_ONCE

    try:
        session = get_session()
    except RobotUnavailableError as exc:
        return (
            f"Cannot reach the robot ({exc}). Was the agent launched from a shell that "
            "sourced ROS2 + aimdk?"
        )

    from aimdk_msgs.srv import PlayEmoji  # import différé (dispo seulement sur le robot)

    client = session.get_client(PlayEmoji, EMOJI_SRV)
    req = PlayEmoji.Request()
    req.emotion_id = emotion_id
    req.mode = mode
    req.priority = int(priority)

    try:
        result = session.call_service(client, req, service_name=EMOJI_SRV)
    except RobotUnavailableError:
        return (
            f"The robot's face service ({EMOJI_SRV}) did not respond; "
            f"the '{emotion_key}' expression was not shown."
        )

    # La réponse porte un booléen `success` fiable (status.value reste 0 même en succès).
    if getattr(result, "success", False):
        how = " (looping)" if loop else ""
        return f"Done — the robot is showing the '{emotion_key}' expression{how}."
    return (
        f"The '{emotion_key}' expression was rejected by the robot "
        f"(message={getattr(result, 'message', '')!r})."
    )
