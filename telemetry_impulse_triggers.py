"""
Shared-memory telemetry -> Xbox IMPULSE TRIGGER haptics (GameInput backend).

Reads live physics data via Windows named shared memory (no UDP/socket),
using mmap + ctypes with a zero-copy view, and drives the left/right
impulse trigger motors of an Xbox-compatible controller via the native
GameInput API (gameinput.dll / GameInputRedist.dll).

Backend note: on some Xbox-licensed pads (e.g. 8BitDo Ultimate C) the
winsdk (Windows.Gaming.Input) vibration path is silently ignored, while
XInput only reaches the main grip motors. GameInput's SetRumbleState
addresses low/high + left-trigger/right-trigger motors directly, which is
what this script uses: tire slip -> LEFT trigger, grip loss -> RIGHT trigger.

Low-latency design:
  - PhysicsBlock.from_buffer(mm) maps shared memory once (no per-frame copy).
  - GameInput device acquired once via async enumeration; hot loop only
    reads two float fields, scales, clamps, and calls SetRumbleState with a
    reused struct.
  - Absolutely no print() / input() / sleep / heavy I/O inside the hot loop.
"""

import ctypes
import mmap
import time

# ---------------------------------------------------------------------------
# Configurable settings — edit these per simulator.
# ---------------------------------------------------------------------------

# Windows named file-mapping string created by the game.
# Assetto Corsa examples: r"Local\acpmf_physics", r"Local\acpmf_graphics",
# r"Local\acpmf_static". Change to match your sim.
SHM_NAME = r"Local\acpmf_physics"

# Size (bytes) to map. Must cover all configured offsets + 4.
# Keep it small (just what we need) so it also opens when the game's block
# is larger. Increase only if your offsets lie further into the block.
SHM_SIZE = 128

# Byte offsets (from start of shared-memory block) of the float32 channels.
# Change these to match each simulator's physics layout.
# AC/ACC/AC Rally: wheelSlip[4] lives at 56,60,64,68 (FL,FR,RL,RR).
# LEFT trigger watches BOTH front wheels (lockup = max of the two);
# RIGHT trigger watches BOTH rear wheels (slide = whichever breaks first).
TIRE_SLIP_OFFSET_L = 56  # wheelSlip[FL] --+
TIRE_SLIP_OFFSET_R = 60  # wheelSlip[FR] --+-> left impulse trigger (max of |both|)
GRIP_LOSS_OFFSET_L = 64  # wheelSlip[RL] --+
GRIP_LOSS_OFFSET_R = 68  # wheelSlip[RR] --+-> right impulse trigger (max of |both|)

# Expected value range for scaling raw physics values to 0.0-1.0 vibration.
# If your sim outputs 0-100 instead of 0.0-1.0, set these to 100.0.
TIRE_SLIP_MAX = 1.0
GRIP_LOSS_MAX = 1.0

# Intensity = clamp((slip - THRESHOLD) * GAIN / MAX).
# LEFT trigger = FRONT LOCKUP channel: a locked tire reads ~1.0 slip ratio,
# far above cornering scrub, so threshold 0.70 stays silent until a front
# wheel actually locks. 0.7 -> 0, 0.95 -> 50%, 1.2+ -> full.
TIRE_SLIP_THRESHOLD = 0.70
TIRE_SLIP_GAIN = 2.0
# RIGHT trigger = rear slide/wheelspin channel, MAX tune: threshold 0.08
# with 8x gain. 0.08 -> 0, 0.1 -> 16%, 0.15 -> 56%, 0.2+ -> full.
# Warning: hard driving just below the limit will buzz constantly.
GRIP_LOSS_THRESHOLD = 0.08
GRIP_LOSS_GAIN = 8.0

# Every N frames, re-check the pad is still connected (handles reconnects).
# Cheap: one counter check per frame, one light status call per N frames.
PAD_CHECK_FRAMES = 1024

# How long (seconds) to wait between retries while waiting for the game /
# controller during setup. NOT used inside the hot loop.
SETUP_RETRY_DELAY = 1.0

# ---------------------------------------------------------------------------
# ctypes mapping of the shared-memory block.
# Padding sizes are derived from the OFFSET variables above, so editing the
# offsets at the top automatically remaps the fields.
# _pack_ = 1 guarantees no compiler-inserted padding.
#
# NOTE: this layout assumes TIRE_SLIP_OFFSET_L <= TIRE_SLIP_OFFSET_R <=
# GRIP_LOSS_OFFSET_L <= GRIP_LOSS_OFFSET_R (the default). If your sim orders
# them differently, reorder the field groups smallest-offset-first.
# ---------------------------------------------------------------------------

_FLOAT_SIZE = ctypes.sizeof(ctypes.c_float)


class PhysicsBlock(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("_pad0", ctypes.c_byte * TIRE_SLIP_OFFSET_L),
        ("tire_slip_l", ctypes.c_float),  # front-left slip --+
        ("_pad01", ctypes.c_byte * (TIRE_SLIP_OFFSET_R - TIRE_SLIP_OFFSET_L - _FLOAT_SIZE)),
        ("tire_slip_r", ctypes.c_float),  # front-right slip -+-> left trigger (max of |both|)
        ("_pad1", ctypes.c_byte * (GRIP_LOSS_OFFSET_L - TIRE_SLIP_OFFSET_R - _FLOAT_SIZE)),
        ("grip_loss_l", ctypes.c_float),  # rear-left slip --+
        ("_pad23", ctypes.c_byte * (GRIP_LOSS_OFFSET_R - GRIP_LOSS_OFFSET_L - _FLOAT_SIZE)),
        ("grip_loss_r", ctypes.c_float),  # rear-right slip -+-> right trigger (max of |both|)
        ("_pad_tail", ctypes.c_byte * (SHM_SIZE - GRIP_LOSS_OFFSET_R - _FLOAT_SIZE)),
    ]


# ---------------------------------------------------------------------------
# Minimal GameInput bindings (ctypes, stdlib only).
# Vtable indices differ between the v3 ABI (GameInputRedist.dll) and the
# legacy v0 ABI (gameinput.dll); both are handled.
# ---------------------------------------------------------------------------

class _GUID(ctypes.Structure):
    _fields_ = [("Data1", ctypes.c_uint32),
                ("Data2", ctypes.c_uint16),
                ("Data3", ctypes.c_uint16),
                ("Data4", ctypes.c_ubyte * 8)]


class RumbleParams(ctypes.Structure):
    _fields_ = [("lowFrequency", ctypes.c_float),
                ("highFrequency", ctypes.c_float),
                ("leftTrigger", ctypes.c_float),
                ("rightTrigger", ctypes.c_float)]


_IID_IGAMEINPUT = _GUID(0x20EFC1C7, 0x5D9A, 0x43BA,
                        (ctypes.c_ubyte * 8)(0xB2, 0x6F, 0xB8, 0x07, 0xFA, 0x48, 0x60, 0x9C))

_KIND_GAMEPAD = 0x00040000
_STATUS_CONNECTED = 0x00000001
_ENUM_ASYNC = 1

# Vtable slots (after IUnknown QueryInterface/AddRef/Release = 0,1,2).
_IDX = {
    "v3": {"reg": 8, "rumble": 9, "status": 5, "unreg": 12},
    "v0": {"reg": 9, "rumble": 10, "status": 4, "unreg": 13},
}


def _vfunc(this, index, restype, argtypes):
    vtbl = ctypes.cast(this, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p)))
    proto = ctypes.WINFUNCTYPE(restype, ctypes.c_void_p, *argtypes)
    return proto(vtbl.contents[index])


def _addref(this):
    return _vfunc(this, 1, ctypes.c_ulong, [])(this)


def _release(this):
    return _vfunc(this, 2, ctypes.c_ulong, [])(this)


class GameInputPad:
    """Owns IGameInput, device-callback registration, and trigger output."""

    def __init__(self):
        self.abi = None
        self.gi = None
        self.token = ctypes.c_uint64(0)
        self._dev = None
        self._cb = None
        self._rumble = RumbleParams(0.0, 0.0, 0.0, 0.0)
        self._init_api()

    def _init_api(self):
        try:
            lib = ctypes.WinDLL("GameInputRedist.dll")
            lib.GameInputInitialize.argtypes = (
                ctypes.POINTER(_GUID), ctypes.POINTER(ctypes.c_void_p))
            lib.GameInputInitialize.restype = ctypes.c_long
            out = ctypes.c_void_p()
            if lib.GameInputInitialize(ctypes.byref(_IID_IGAMEINPUT),
                                       ctypes.byref(out)) == 0:
                self.lib, self.gi, self.abi = lib, out.value, "v3"
                return
        except OSError:
            pass
        lib = ctypes.WinDLL("gameinput.dll")
        lib.GameInputCreate.argtypes = (ctypes.POINTER(ctypes.c_void_p),)
        lib.GameInputCreate.restype = ctypes.c_long
        out = ctypes.c_void_p()
        if lib.GameInputCreate(ctypes.byref(out)) != 0:
            raise SystemExit("GameInput init failed")
        self.lib, self.gi, self.abi = lib, out.value, "v0"

    def _idx(self, name):
        return _IDX[self.abi][name]

    def start(self):
        """Register for gamepad connect events (async => also enumerates)."""
        cbtype = ctypes.WINFUNCTYPE(None, ctypes.c_uint64, ctypes.c_void_p,
                                    ctypes.c_void_p, ctypes.c_uint64,
                                    ctypes.c_int, ctypes.c_int)

        def _on_device(token, ctx, device, ts, cur, prev):
            if device and (cur & _STATUS_CONNECTED):
                _addref(device)
                old, self._dev = self._dev, device
                if old:
                    _release(old)

        self._cb = cbtype(_on_device)  # keep alive
        reg = _vfunc(self.gi, self._idx("reg"), ctypes.c_long,
                     [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_int,
                      ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p])
        hr = reg(self.gi, None, _KIND_GAMEPAD, _STATUS_CONNECTED, _ENUM_ASYNC,
                 None, ctypes.cast(self._cb, ctypes.c_void_p),
                 ctypes.byref(self.token))
        if hr != 0:
            raise SystemExit(f"RegisterDeviceCallback failed: 0x{hr & 0xFFFFFFFF:08X}")

    def wait_for_pad(self):
        """Block (setup path only) until a gamepad is present."""
        while self._dev is None:
            print("No gamepad seen by GameInput. Connect it...")
            time.sleep(SETUP_RETRY_DELAY)
        print("Controller connected. Forwarding slip -> impulse triggers. Ctrl+C to stop.")

    def connected(self) -> bool:
        """Lightweight still-connected check for the periodic hot-loop probe."""
        if self._dev is None:
            return False
        get_status = _vfunc(self._dev, self._idx("status"), ctypes.c_int, [])
        try:
            return bool(get_status(self._dev) & _STATUS_CONNECTED)
        except Exception:
            return False

    def set_triggers(self, left: float, right: float) -> None:
        self._rumble.leftTrigger = left
        self._rumble.rightTrigger = right
        _vfunc(self._dev, self._idx("rumble"), None,
               [ctypes.c_void_p])(self._dev, ctypes.byref(self._rumble))

    def stop(self):
        """Zero rumble, unregister, release (cleanup path only)."""
        try:
            if self._dev is not None:
                self.set_triggers(0.0, 0.0)
        except Exception:
            pass
        try:
            if self.abi == "v3":
                unreg = _vfunc(self.gi, self._idx("unreg"), ctypes.c_bool,
                               [ctypes.c_uint64])
                unreg(self.gi, self.token.value)
            else:
                unreg = _vfunc(self.gi, self._idx("unreg"), ctypes.c_bool,
                               [ctypes.c_uint64, ctypes.c_uint64])
                unreg(self.gi, self.token.value, 1000000)
        except Exception:
            pass
        try:
            if self._dev is not None:
                _release(self._dev)
                self._dev = None
            _release(self.gi)
        except Exception:
            pass


def main() -> None:
    required = max(TIRE_SLIP_OFFSET_L, TIRE_SLIP_OFFSET_R,
                   GRIP_LOSS_OFFSET_L, GRIP_LOSS_OFFSET_R) + _FLOAT_SIZE
    if SHM_SIZE < required:
        print(f"SHM_SIZE ({SHM_SIZE}) too small, need >= {required} for offsets. Exiting.")
        return
    assert PhysicsBlock.tire_slip_l.offset == TIRE_SLIP_OFFSET_L, "tire_slip_l misaligned"
    assert PhysicsBlock.tire_slip_r.offset == TIRE_SLIP_OFFSET_R, "tire_slip_r misaligned"
    assert PhysicsBlock.grip_loss_l.offset == GRIP_LOSS_OFFSET_L, "grip_loss_l misaligned"
    assert PhysicsBlock.grip_loss_r.offset == GRIP_LOSS_OFFSET_R, "grip_loss_r misaligned"

    print(f"Waiting for shared memory '{SHM_NAME}' ... (start the game)")
    mm = None
    while mm is None:
        try:
            mm = mmap.mmap(-1, SHM_SIZE, tagname=SHM_NAME, access=mmap.ACCESS_WRITE)
        except (FileNotFoundError, OSError):
            time.sleep(SETUP_RETRY_DELAY)
    print(f"Attached to '{SHM_NAME}' ({SHM_SIZE} bytes).")
    print(f"Front slip @ bytes {TIRE_SLIP_OFFSET_L}/{TIRE_SLIP_OFFSET_R}, "
          f"rear slip @ bytes {GRIP_LOSS_OFFSET_L}/{GRIP_LOSS_OFFSET_R}")

    pad = GameInputPad()
    pad.start()
    pad.wait_for_pad()

    # Zero-copy view: phys.* read game memory directly.
    phys = PhysicsBlock.from_buffer(mm)

    # Cache everything as locals so the hot loop avoids attribute lookups.
    _phys = phys
    _set = pad.set_triggers
    _ok = pad.connected
    slip_scale = TIRE_SLIP_GAIN / (TIRE_SLIP_MAX if TIRE_SLIP_MAX != 0 else 1.0)
    grip_scale = GRIP_LOSS_GAIN / (GRIP_LOSS_MAX if GRIP_LOSS_MAX != 0 else 1.0)
    slip_thresh = TIRE_SLIP_THRESHOLD
    grip_thresh = GRIP_LOSS_THRESHOLD
    check_n = PAD_CHECK_FRAMES if PAD_CHECK_FRAMES > 0 else 1024

    try:
        while True:  # reconnect loop (setup path on disconnect)
            pad.wait_for_pad()
            # ---------------- Hot loop: no prints / no sleep / no heavy I/O ----------------
            frame = 0
            while True:
                # Left = front lockup: loudest of the two front wheels (|.| so
                # either slide direction or lockup polarity registers).
                front_l = _phys.tire_slip_l
                if front_l < 0.0:
                    front_l = -front_l
                front_r = _phys.tire_slip_r
                if front_r < 0.0:
                    front_r = -front_r
                if front_r > front_l:
                    front_l = front_r
                tire_slip = (front_l - slip_thresh) * slip_scale
                # Right = rear slide: loudest of the two rear wheels.
                rear_l = _phys.grip_loss_l
                if rear_l < 0.0:
                    rear_l = -rear_l
                rear_r = _phys.grip_loss_r
                if rear_r < 0.0:
                    rear_r = -rear_r
                if rear_r > rear_l:
                    rear_l = rear_r
                grip_loss = (rear_l - grip_thresh) * grip_scale
                if tire_slip < 0.0:
                    tire_slip = 0.0
                elif tire_slip > 1.0:
                    tire_slip = 1.0
                if grip_loss < 0.0:
                    grip_loss = 0.0
                elif grip_loss > 1.0:
                    grip_loss = 1.0
                _set(tire_slip, grip_loss)
                frame += 1
                if frame >= check_n:
                    frame = 0
                    if not _ok():
                        break  # pad lost -> back to reconnect wait
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup lives outside the hot loop — prints OK here.
        pad.stop()
        try:
            if mm is not None:
                mm.close()
        except Exception:
            pass
        print("\nStopped. Triggers zeroed, shared memory released.")


if __name__ == "__main__":
    main()
