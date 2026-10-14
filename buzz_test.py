"""Buzz test: full-strength impulse triggers for 2s, then off."""
import time

from winsdk.windows.gaming.input import Gamepad, GamepadVibration

pad = None
for _ in range(5):
    if Gamepad.gamepads.size > 0:
        pad = Gamepad.gamepads.get_at(0)
        break
    time.sleep(1.0)
if pad is None:
    print("No controller seen - connect it first")
    raise SystemExit(1)

v = GamepadVibration()
v.left_motor = 0.0
v.right_motor = 0.0
v.left_trigger = 1.0
v.right_trigger = 1.0
pad.vibration = v
print("BUZZING both impulse triggers at 100% for 2s...")
time.sleep(2.0)
pad.vibration = GamepadVibration()
print("off - did you feel it in the triggers?")
