"""XInput buzz test: full-strength main motors for 2s, then off."""
import ctypes
import time


class XINPUT_VIBRATION(ctypes.Structure):
    _fields_ = [("wLeftMotorSpeed", ctypes.c_ushort),
                ("wRightMotorSpeed", ctypes.c_ushort)]


def load_xinput():
    for dll in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            lib = ctypes.WinDLL(dll)
            lib.XInputSetState.argtypes = (ctypes.c_uint,
                                           ctypes.POINTER(XINPUT_VIBRATION))
            lib.XInputSetState.restype = ctypes.c_uint
            print(f"using {dll}", flush=True)
            return lib
        except OSError:
            continue
    raise SystemExit("no XInput DLL found")


xi = load_xinput()
vib = XINPUT_VIBRATION(65535, 65535)
found = False
for idx in range(4):
    rc = xi.XInputSetState(idx, ctypes.byref(vib))
    print(f"pad {idx}: XInputSetState -> {rc} (0 = success)", flush=True)
    if rc == 0:
        found = True
        break
if not found:
    print("NO XINPUT PAD ACCEPTED VIBRATION - check connection")
    raise SystemExit(1)

print("BUZZING both main motors at 100% for 2s...", flush=True)
time.sleep(2.0)
xi.XInputSetState(idx, ctypes.byref(XINPUT_VIBRATION(0, 0)))
print("off - did you feel strong grip rumble?", flush=True)
