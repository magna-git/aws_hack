import time
from tools.preset_motion import preset_motion
print("A raise/right :", preset_motion(motion="raise", arm="right"))
print("B wave/right  :", preset_motion(motion="wave",  arm="right"))
print("C wave/left   :", preset_motion(motion="wave",  arm="left"))
