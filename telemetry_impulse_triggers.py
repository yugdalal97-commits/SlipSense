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
import sys
import time

# Trigger feel presets — "tarmac" is your current perfect tune, "dirt" is
# calmer for loose surfaces where baseline slip is always higher.
# Switch with: SURFACE_MODE below, or: python script.py dirt
SURFACE_MODE = "tarmac"  # "tarmac" | "dirt"
TRIGGER_PRESETS = {
    "tarmac": {
        "tire_thresh": 0.25, "tire_gain": 4.0,   # left: lockup, silent in turns
        "grip_thresh": 0.08, "grip_gain": 8.0,   # right: rear slide, MAX tune
    },
    "dirt": {
        "tire_thresh": 0.35, "tire_gain": 3.0,   # left: lockup, calmer for dirt
        "grip_thresh": 0.35, "grip_gain": 3.0,   # right: slides need real slip
    },
}

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

# Pedal channels (0.0-1.0) for gating trigger logic.
BRAKE_OFFSET = 8   # float32 brake -> suppresses right (slide) trigger while braking
# Extra channels for the two main grip motors (event-only immersion).
RPM_OFFSET = 20    # int32 rpms -> high-frequency motor (redline + shift thump)
GEAR_OFFSET = 16   # int32 gear -> shift detection (N=0/R=-1, else gear number)
VERT_G_OFFSET = 52  # float32 vertical G -> low-frequency motor (jump landings)
LAT_G_OFFSET = 44   # float32 lateral G -> low-frequency motor (hard cornering)

# Engine: linear with revs, so idle gives a faint tremor and redline screams.
# MAX_RPM should match your car (rally cars ~7000-8500).
MAX_RPM = 8000.0
RPM_GAIN = 1.0
# Redline warning: silent until revs pass REDLINE_FRAC of MAX_RPM, then ramps
# to full at the limiter. 0.92 -> first tremor at 92%, pegged at 100%.
REDLINE_FRAC = 0.92
# Shift thump: full-strength burst on every gear change, this long (seconds).
SHIFT_PULSE_SEC = 0.15
# Landing thump: Rally reports vertical G WITHOUT gravity, so flat ground
# reads 0.0 (not 1.0 like AC1). Only upward spikes count: gentle driving
# stays under ~0.3, hard acceleration squat reaches ~0.8, landings spike far
# past 1.0. 0.45 -> 0, 0.7 -> 50%, 0.95+ -> full slam.
BUMP_THRESHOLD = 0.45
BUMP_GAIN = 2.0
# Hard-cornering voice (same motor, louder wins): lateral G past this means
# the car is really leaning on the tires. Gentle bends stay under ~0.5;
# committed rally corners spike past 1.0. 0.8 -> 0, 1.05 -> 50%, 1.3+ -> full.
LAT_G_THRESHOLD = 0.80
LAT_G_GAIN = 2.0

# Expected value range for scaling raw physics values to 0.0-1.0 vibration.
# If your sim outputs 0-100 instead of 0.0-1.0, set these to 100.0.
TIRE_SLIP_MAX = 1.0
GRIP_LOSS_MAX = 1.0

# Intensity = clamp((slip - THRESHOLD) * GAIN / MAX).
# Active values come from TRIGGER_PRESETS[SURFACE_MODE] (see top of file).
# Tarmac:
#   LEFT = front lockup: 0.25 -> 0, 0.375 -> 50%, 0.5+ -> full.
#   RIGHT = rear slide MAX: 0.08 -> 0, 0.15 -> 56%, 0.2+ -> full.
# Dirt (calmer for loose surfaces):
#   LEFT = 0.35 -> 0, 0.52 -> 50%, 0.68+ -> full.
#   RIGHT = 0.35 -> 0, 0.52 -> 50%, 0.68+ -> full.
TIRE_SLIP_THRESHOLD = TRIGGER_PRESETS[SURFACE_MODE]["tire_thresh"]
TIRE_SLIP_GAIN = TRIGGER_PRESETS[SURFACE_MODE]["tire_gain"]
GRIP_LOSS_THRESHOLD = TRIGGER_PRESETS[SURFACE_MODE]["grip_thresh"]
GRIP_LOSS_GAIN = TRIGGER_PRESETS[SURFACE_MODE]["grip_gain"]

# Pause/mute: packetId (byte 0) ticks every physics update. If it stops
# changing the sim is paused/in menus with frozen values, so mute all motors
# after this many seconds of stale data instead of holding the last buzz.
STALE_MUTE_SEC = 0.5
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
# NOTE: this layout assumes ascending offsets (the default). If your sim
# orders them differently, reorder the field groups smallest-offset-first.
# ---------------------------------------------------------------------------

_FLOAT_SIZE = ctypes.sizeof(ctypes.c_float)
_INT_SIZE = ctypes.sizeof(ctypes.c_int)


class PhysicsBlock(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("packet_id", ctypes.c_int),  # byte 0: ticks every physics update
        ("_pad_gas", ctypes.c_byte * (BRAKE_OFFSET - 0 - _INT_SIZE)),
        ("brake", ctypes.c_float),  # byte BRAKE_OFFSET: 0.0-1.0 pedal
        ("_pad0pre", ctypes.c_byte * (GEAR_OFFSET - BRAKE_OFFSET - _FLOAT_SIZE)),
        ("gear", ctypes.c_int),  # byte GEAR_OFFSET -> shift-thump detection
        ("_pad0a", ctypes.c_byte * (RPM_OFFSET - GEAR_OFFSET - _INT_SIZE)),
        ("rpm", ctypes.c_int),  # byte RPM_OFFSET -> high-freq motor
        ("_pad0b", ctypes.c_byte * (LAT_G_OFFSET - RPM_OFFSET - _INT_SIZE)),
        ("g_lat", ctypes.c_float),  # lateral G -> low-freq motor (hard turns)
        ("_pad0b2", ctypes.c_byte * (VERT_G_OFFSET - LAT_G_OFFSET - _FLOAT_SIZE)),
        ("vert_g", ctypes.c_float),  # vertical G (0.0 = flat) -> low-freq motor
        ("_pad0c", ctypes.c_byte * (TIRE_SLIP_OFFSET_L - VERT_G_OFFSET - _FLOAT_SIZE)),
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

    def set_all(self, low: float, high: float, left: float, right: float) -> None:
        self._rumble.lowFrequency = low
        self._rumble.highFrequency = high
        self._rumble.leftTrigger = left
        self._rumble.rightTrigger = right
        _vfunc(self._dev, self._idx("rumble"), None,
               [ctypes.c_void_p])(self._dev, ctypes.byref(self._rumble))

    def stop(self):
        """Zero all motors, unregister, release (cleanup path only)."""
        try:
            if self._dev is not None:
                self.set_all(0.0, 0.0, 0.0, 0.0)
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
    assert PhysicsBlock.packet_id.offset == 0, "packet_id misaligned"
    assert PhysicsBlock.brake.offset == BRAKE_OFFSET, "brake misaligned"
    assert PhysicsBlock.gear.offset == GEAR_OFFSET, "gear misaligned"
    assert PhysicsBlock.rpm.offset == RPM_OFFSET, "rpm misaligned"
    assert PhysicsBlock.vert_g.offset == VERT_G_OFFSET, "vert_g misaligned"
    assert PhysicsBlock.g_lat.offset == LAT_G_OFFSET, "g_lat misaligned"
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

    # Resolve trigger preset: CLI arg wins ("python script.py dirt"),
    # otherwise SURFACE_MODE from the top of the file.
    mode = SURFACE_MODE.lower() if isinstance(SURFACE_MODE, str) else "tarmac"
    for _arg in sys.argv[1:]:
        _a = _arg.lower().lstrip("-")
        if _a in TRIGGER_PRESETS:
            mode = _a
            break
    if mode not in TRIGGER_PRESETS:
        print(f"Unknown surface mode '{mode}', falling back to 'tarmac'.")
        mode = "tarmac"
    _preset = TRIGGER_PRESETS[mode]
    print(f"Surface mode: {mode} "
          f"(left { _preset['tire_thresh']}/{_preset['tire_gain']}x, "
          f"right {_preset['grip_thresh']}/{_preset['grip_gain']}x)")

    # Cache everything as locals so the hot loop avoids attribute lookups.
    _phys = phys
    _set = pad.set_all
    _ok = pad.connected
    slip_scale = _preset["tire_gain"] / (TIRE_SLIP_MAX if TIRE_SLIP_MAX != 0 else 1.0)
    grip_scale = _preset["grip_gain"] / (GRIP_LOSS_MAX if GRIP_LOSS_MAX != 0 else 1.0)
    rpm_scale = RPM_GAIN / (MAX_RPM if MAX_RPM != 0 else 8000.0)
    red_frac = REDLINE_FRAC
    red_span = 1.0 - REDLINE_FRAC if REDLINE_FRAC < 1.0 else 0.08
    shift_len = SHIFT_PULSE_SEC
    bump_thresh = BUMP_THRESHOLD
    bump_gain = BUMP_GAIN
    lat_thresh = LAT_G_THRESHOLD
    lat_gain = LAT_G_GAIN
    slip_thresh = _preset["tire_thresh"]
    grip_thresh = _preset["grip_thresh"]
    _mono = time.monotonic
    last_gear = None
    pulse_end = 0.0
    last_pid = None
    pid_time = 0.0
    stale_sec = STALE_MUTE_SEC
    check_n = PAD_CHECK_FRAMES if PAD_CHECK_FRAMES > 0 else 1024

    try:
        while True:  # reconnect loop (setup path on disconnect)
            pad.wait_for_pad()
            # ---------------- Hot loop: no prints / no sleep / no heavy I/O ----------------
            frame = 0
            while True:
                now = _mono()
                # Pause detect: frozen packet counter = frozen physics.
                pid = _phys.packet_id
                if pid != last_pid:
                    last_pid = pid
                    pid_time = now
                if now - pid_time > stale_sec:
                    _set(0.0, 0.0, 0.0, 0.0)
                else:
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
                    # Brake gate: hard braking unloads the rears so their slip
                    # spikes too — mute the slide channel while braking so a
                    # front lockup is left-trigger-only. Full signal at light
                    # pedal (<=0.05), fully muted at >=0.30. Lift-off (no pedals)
                    # still warns.
                    _brake = _phys.brake
                    if _brake > 0.05:
                        _gate = 1.0 - (_brake - 0.05) / 0.25
                        if _gate <= 0.0:
                            grip_loss = 0.0
                        else:
                            grip_loss = grip_loss * _gate
                    if tire_slip < 0.0:
                        tire_slip = 0.0
                    elif tire_slip > 1.0:
                        tire_slip = 1.0
                    if grip_loss < 0.0:
                        grip_loss = 0.0
                    elif grip_loss > 1.0:
                        grip_loss = 1.0
                    # Event layer (normally silent):
                    # high-freq = redline warning + gear-shift thump,
                    # low-freq = louder of landing slam vs. hard cornering.
                    gear_now = _phys.gear
                    if last_gear is None:
                        last_gear = gear_now
                    elif gear_now != last_gear:
                        last_gear = gear_now
                        pulse_end = now + shift_len
                    engine = 0.0
                    frac = _phys.rpm * rpm_scale
                    if frac > red_frac:
                        engine = (frac - red_frac) / red_span
                        if engine > 1.0:
                            engine = 1.0
                    if now < pulse_end:
                        engine = 1.0
                    bump = _phys.vert_g - bump_thresh
                    if bump < 0.0:
                        bump = 0.0
                    gforce = bump * bump_gain
                    if gforce > 1.0:
                        gforce = 1.0
                    lat = _phys.g_lat
                    if lat < 0.0:
                        lat = -lat
                    corner = (lat - lat_thresh) * lat_gain
                    if corner < 0.0:
                        corner = 0.0
                    elif corner > 1.0:
                        corner = 1.0
                    if corner > gforce:
                        gforce = corner
                    _set(gforce, engine, tire_slip, grip_loss)
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
