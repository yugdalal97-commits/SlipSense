"""
Impulse-trigger haptics control panel (Assetto Corsa Rally).

Same engine as telemetry_impulse_triggers.py — shared-memory physics via
mmap + ctypes, trigger/grip rumble via native GameInput — but running in a
background thread under a Tkinter UI instead of a console script.

UI stays in the main thread; the hot loop never touches widgets. Tunables
live in a shared dict that the loop re-reads every 256 frames, so sliders
apply live with zero added per-frame cost.

Run:
    python "C:\\impulse tiggers\\haptics_ui.py"
No pip packages needed (stdlib only: ctypes, mmap, threading, tkinter).
"""

import ctypes
import mmap
import threading
import time
import tkinter as tk
from tkinter import ttk

# ---------------------------------------------------------------------------
# Fixed shared-memory layout (Assetto Corsa physics block — not user-tunable).
# ---------------------------------------------------------------------------

SHM_NAME = r"Local\acpmf_physics"
SHM_SIZE = 128

BRAKE_OFFSET = 8     # float32 brake pedal 0.0-1.0
GEAR_OFFSET = 16     # int32 gear
RPM_OFFSET = 20      # int32 rpms
LAT_G_OFFSET = 44    # float32 lateral G
VERT_G_OFFSET = 52   # float32 vertical G (0.0 = flat, Rally excludes gravity)
TIRE_SLIP_OFFSET_L = 56  # wheelSlip[FL] -> left trigger (max of both fronts)
TIRE_SLIP_OFFSET_R = 60  # wheelSlip[FR] -/
GRIP_LOSS_OFFSET_L = 64  # wheelSlip[RL] -> right trigger (max of both rears)
GRIP_LOSS_OFFSET_R = 68  # wheelSlip[RR] -/

# ---------------------------------------------------------------------------
# Defaults + surface presets (tarmac = current proven tune).
# ---------------------------------------------------------------------------

DEFAULTS = {
    "tire_thresh": 0.25, "tire_gain": 4.0,    # left: front lockup
    "grip_thresh": 0.08, "grip_gain": 8.0,     # right: rear slide (MAX)
    "brake_gate": True,                        # mute slide channel while braking
    "max_rpm": 8000.0, "redline_frac": 0.92, "shift_pulse": 0.15,
    "bump_thresh": 0.45, "bump_gain": 2.0,     # low motor: landings
    "lat_thresh": 0.80, "lat_gain": 2.0,       # low motor: hard cornering
}

TRIGGER_PRESETS = {
    "tarmac": {"tire_thresh": 0.25, "tire_gain": 4.0,
               "grip_thresh": 0.08, "grip_gain": 8.0},
    "dirt": {"tire_thresh": 0.35, "tire_gain": 3.0,
             "grip_thresh": 0.35, "grip_gain": 3.0},
}

STALE_MUTE_SEC = 0.5
PAD_CHECK_FRAMES = 1024
CONFIG_REFRESH_FRAMES = 256
METER_EVERY_FRAMES = 64

# ---------------------------------------------------------------------------
# ctypes mapping of the shared-memory block (zero-copy view).
# ---------------------------------------------------------------------------

_FLOAT_SIZE = ctypes.sizeof(ctypes.c_float)
_INT_SIZE = ctypes.sizeof(ctypes.c_int)


class PhysicsBlock(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("packet_id", ctypes.c_int),
        ("_pad_gas", ctypes.c_byte * (BRAKE_OFFSET - 0 - _INT_SIZE)),
        ("brake", ctypes.c_float),
        ("_pad0pre", ctypes.c_byte * (GEAR_OFFSET - BRAKE_OFFSET - _FLOAT_SIZE)),
        ("gear", ctypes.c_int),
        ("_pad0a", ctypes.c_byte * (RPM_OFFSET - GEAR_OFFSET - _INT_SIZE)),
        ("rpm", ctypes.c_int),
        ("_pad0b", ctypes.c_byte * (LAT_G_OFFSET - RPM_OFFSET - _INT_SIZE)),
        ("g_lat", ctypes.c_float),
        ("_pad0b2", ctypes.c_byte * (VERT_G_OFFSET - LAT_G_OFFSET - _FLOAT_SIZE)),
        ("vert_g", ctypes.c_float),
        ("_pad0c", ctypes.c_byte * (TIRE_SLIP_OFFSET_L - VERT_G_OFFSET - _FLOAT_SIZE)),
        ("tire_slip_l", ctypes.c_float),
        ("_pad01", ctypes.c_byte * (TIRE_SLIP_OFFSET_R - TIRE_SLIP_OFFSET_L - _FLOAT_SIZE)),
        ("tire_slip_r", ctypes.c_float),
        ("_pad1", ctypes.c_byte * (GRIP_LOSS_OFFSET_L - TIRE_SLIP_OFFSET_R - _FLOAT_SIZE)),
        ("grip_loss_l", ctypes.c_float),
        ("_pad23", ctypes.c_byte * (GRIP_LOSS_OFFSET_R - GRIP_LOSS_OFFSET_L - _FLOAT_SIZE)),
        ("grip_loss_r", ctypes.c_float),
        ("_pad_tail", ctypes.c_byte * (SHM_SIZE - GRIP_LOSS_OFFSET_R - _FLOAT_SIZE)),
    ]


# ---------------------------------------------------------------------------
# Minimal GameInput bindings (ctypes, stdlib only).
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
            raise RuntimeError("GameInput init failed")
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
            raise RuntimeError(f"RegisterDeviceCallback failed: 0x{hr & 0xFFFFFFFF:08X}")

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


# ---------------------------------------------------------------------------
# Haptics engine thread — hot loop never touches widgets or prints.
# ---------------------------------------------------------------------------

class HapticsEngine(threading.Thread):
    def __init__(self, config):
        super().__init__(daemon=True)
        self.config = config                      # shared tunables (UI writes)
        self.enabled = threading.Event()          # toggle: haptics on/off
        self.enabled.set()                        # start ON like the script
        self.stop_flag = threading.Event()
        self.status = {"game": False, "pad": False, "error": ""}
        self.meter = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0,
                      "left": 0.0, "right": 0.0, "low": 0.0, "high": 0.0}

    def _snap_config(self):
        c = self.config
        return (c["tire_thresh"], c["tire_gain"] / 1.0,
                c["grip_thresh"], c["grip_gain"] / 1.0,
                c["brake_gate"], c["max_rpm"] if c["max_rpm"] != 0 else 8000.0,
                c["redline_frac"], c["shift_pulse"],
                c["bump_thresh"], c["bump_gain"],
                c["lat_thresh"], c["lat_gain"])

    def run(self):
        mm = None
        pad = None
        try:
            while mm is None and not self.stop_flag.is_set():
                try:
                    mm = mmap.mmap(-1, SHM_SIZE, tagname=SHM_NAME,
                                   access=mmap.ACCESS_WRITE)
                except (FileNotFoundError, OSError):
                    time.sleep(0.25)
            if mm is None:
                return
            self.status["game"] = True
            pad = GameInputPad()
            pad.start()
            while pad._dev is None and not self.stop_flag.is_set():
                time.sleep(0.25)
            if pad._dev is None:
                return
            self.status["pad"] = True

            phys = PhysicsBlock.from_buffer(mm)
            _phys, _meter = phys, self.meter
            _ok = pad.connected

            (slip_thresh, slip_scale, grip_thresh, grip_scale, brake_gate,
             max_rpm, red_frac, shift_len,
             bump_thresh, bump_gain, lat_thresh, lat_gain) = self._snap_config()
            rpm_scale_denom = max_rpm
            red_span = 1.0 - red_frac if red_frac < 1.0 else 0.08

            _mono = time.monotonic
            last_gear = None
            pulse_end = 0.0
            last_pid = None
            pid_time = 0.0
            zeroed = False
            frame = 0

            while not self.stop_flag.is_set():
                if not self.enabled.is_set():
                    if not zeroed:
                        try:
                            pad.set_all(0.0, 0.0, 0.0, 0.0)
                        except Exception:
                            pass
                        zeroed = True
                        _meter.update(fl=0.0, fr=0.0, rl=0.0, rr=0.0,
                                      left=0.0, right=0.0, low=0.0, high=0.0)
                    last_pid = _phys.packet_id
                    pid_time = _mono()
                    time.sleep(0.05)
                    continue
                zeroed = False

                now = _mono()
                pid = _phys.packet_id
                if pid != last_pid:
                    last_pid = pid
                    pid_time = now
                if now - pid_time > STALE_MUTE_SEC:
                    try:
                        pad.set_all(0.0, 0.0, 0.0, 0.0)
                    except Exception:
                        pass
                else:
                    front_l = _phys.tire_slip_l
                    if front_l < 0.0:
                        front_l = -front_l
                    front_r = _phys.tire_slip_r
                    if front_r < 0.0:
                        front_r = -front_r
                    if front_r > front_l:
                        front_l = front_r
                    tire_slip = (front_l - slip_thresh) * slip_scale
                    rear_l = _phys.grip_loss_l
                    if rear_l < 0.0:
                        rear_l = -rear_l
                    rear_r = _phys.grip_loss_r
                    if rear_r < 0.0:
                        rear_r = -rear_r
                    if rear_r > rear_l:
                        rear_l = rear_r
                    grip_loss = (rear_l - grip_thresh) * grip_scale
                    if brake_gate:
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
                    gear_now = _phys.gear
                    if last_gear is None:
                        last_gear = gear_now
                    elif gear_now != last_gear:
                        last_gear = gear_now
                        pulse_end = now + shift_len
                    engine = 0.0
                    frac = _phys.rpm / rpm_scale_denom
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
                    try:
                        pad.set_all(gforce, engine, tire_slip, grip_loss)
                    except Exception:
                        pass
                    if frame % METER_EVERY_FRAMES == 0:
                        _meter.update(fl=_phys.tire_slip_l, fr=_phys.tire_slip_r,
                                      rl=_phys.grip_loss_l, rr=_phys.grip_loss_r,
                                      left=tire_slip, right=grip_loss,
                                      low=gforce, high=engine)
                frame += 1
                if frame >= PAD_CHECK_FRAMES:
                    frame = 0
                    if not _ok():
                        self.status["pad"] = False
                        while pad._dev is None and not self.stop_flag.is_set():
                            time.sleep(0.25)
                        if self.stop_flag.is_set():
                            break
                        self.status["pad"] = True
                if frame % CONFIG_REFRESH_FRAMES == 0:
                    (slip_thresh, slip_scale, grip_thresh, grip_scale, brake_gate,
                     max_rpm, red_frac, shift_len,
                     bump_thresh, bump_gain, lat_thresh, lat_gain) = self._snap_config()
                    rpm_scale_denom = max_rpm
                    red_span = 1.0 - red_frac if red_frac < 1.0 else 0.08
        except Exception as e:  # setup path only — never from the hot loop
            self.status["error"] = str(e)
        finally:
            try:
                if pad is not None:
                    pad.stop()
            except Exception:
                pass
            try:
                if mm is not None:
                    mm.close()
            except Exception:
                pass
            self.status["game"] = False
            self.status["pad"] = False


# ---------------------------------------------------------------------------
# Tkinter control panel (main thread).
# ---------------------------------------------------------------------------

class HapticsUI:
    def __init__(self, root):
        self.root = root
        root.title("Impulse Trigger Haptics")
        root.resizable(False, False)

        self.config = dict(DEFAULTS)
        self.engine = HapticsEngine(self.config)
        self._guard = False  # True while applying presets/reset programmatically

        self.mode_var = tk.StringVar(value="tarmac")
        self.vars = {
            "tire_thresh": tk.DoubleVar(value=DEFAULTS["tire_thresh"]),
            "tire_gain": tk.DoubleVar(value=DEFAULTS["tire_gain"]),
            "grip_thresh": tk.DoubleVar(value=DEFAULTS["grip_thresh"]),
            "grip_gain": tk.DoubleVar(value=DEFAULTS["grip_gain"]),
            "max_rpm": tk.DoubleVar(value=DEFAULTS["max_rpm"]),
            "redline_frac": tk.DoubleVar(value=DEFAULTS["redline_frac"]),
            "shift_pulse": tk.DoubleVar(value=DEFAULTS["shift_pulse"]),
            "bump_thresh": tk.DoubleVar(value=DEFAULTS["bump_thresh"]),
            "bump_gain": tk.DoubleVar(value=DEFAULTS["bump_gain"]),
            "lat_thresh": tk.DoubleVar(value=DEFAULTS["lat_thresh"]),
            "lat_gain": tk.DoubleVar(value=DEFAULTS["lat_gain"]),
        }
        self.brake_var = tk.BooleanVar(value=DEFAULTS["brake_gate"])

        for key, var in self.vars.items():
            var.trace_add("write", lambda *a, k=key: self._on_value(k))
        self.brake_var.trace_add("write", lambda *a: self._on_brake())

        self._build()
        self.engine.start()
        self._tick()
        root.protocol("WM_DELETE_WINDOW", self._close)

    # -- layout ---------------------------------------------------------

    def _build(self):
        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")

        self.toggle_btn = ttk.Button(top, text="STOP", command=self._toggle, width=10)
        self.toggle_btn.grid(row=0, column=0, padx=(0, 10))
        self.status_lbl = ttk.Label(top, text="Starting…", width=42)
        self.status_lbl.grid(row=0, column=1, sticky="w")

        surf = ttk.LabelFrame(self.root, text="Surface mode", padding=8)
        surf.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
        for i, (val, txt) in enumerate((("tarmac", "Tarmac"), ("dirt", "Dirt"),
                                        ("custom", "Custom"))):
            ttk.Radiobutton(surf, text=txt, value=val, variable=self.mode_var,
                            command=self._on_mode).grid(row=0, column=i, padx=12)

        trig = ttk.LabelFrame(self.root, text="Impulse triggers", padding=8)
        trig.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 6))
        self._slider(trig, 0, "Left  threshold (front lockup)", "tire_thresh", 0.0, 1.0, 0.01)
        self._slider(trig, 1, "Left  gain", "tire_gain", 0.0, 10.0, 0.1)
        self._slider(trig, 2, "Right threshold (rear slide)", "grip_thresh", 0.0, 1.0, 0.01)
        self._slider(trig, 3, "Right gain", "grip_gain", 0.0, 10.0, 0.1)
        ttk.Checkbutton(trig, text="Mute slide channel while braking",
                        variable=self.brake_var).grid(row=4, column=0, columnspan=2,
                                                      sticky="w", pady=(4, 0))

        grip = ttk.LabelFrame(self.root, text="Grip motors (events only)", padding=8)
        grip.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 6))
        self._slider(grip, 0, "Max RPM (redline = 100%)", "max_rpm", 4000, 12000, 100)
        self._slider(grip, 1, "Redline warning from", "redline_frac", 0.80, 0.99, 0.005)
        self._slider(grip, 2, "Shift thump (sec)", "shift_pulse", 0.0, 0.5, 0.01)
        self._slider(grip, 3, "Landing threshold (vert G)", "bump_thresh", 0.0, 2.0, 0.05)
        self._slider(grip, 4, "Landing gain", "bump_gain", 0.0, 5.0, 0.1)
        self._slider(grip, 5, "Cornering threshold (lat G)", "lat_thresh", 0.0, 2.0, 0.05)
        self._slider(grip, 6, "Cornering gain", "lat_gain", 0.0, 5.0, 0.1)

        mon = ttk.LabelFrame(self.root, text="Live", padding=8)
        mon.grid(row=4, column=0, sticky="ew", padx=10, pady=(0, 6))
        self.mon_lbl = ttk.Label(mon, text="Waiting for game…", font=("TkDefaultFont", 9))
        self.mon_lbl.grid(row=0, column=0, sticky="w")

        bot = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bot.grid(row=5, column=0, sticky="ew")
        ttk.Button(bot, text="Reset to defaults", command=self._reset).grid(row=0, column=0)
        ttk.Label(bot, text="Values apply live — no restart needed.",
                  foreground="gray").grid(row=0, column=1, padx=(12, 0))

    def _slider(self, parent, row, label, key, lo, hi, res):
        ttk.Label(parent, text=label, width=30).grid(row=row, column=0, sticky="w")
        tk.Scale(parent, from_=lo, to=hi, resolution=res, orient="horizontal",
                 length=230, showvalue=True,
                 variable=self.vars[key]).grid(row=row, column=1, sticky="ew")

    # -- behavior --------------------------------------------------------

    def _on_value(self, key):
        try:
            self.config[key] = self.vars[key].get()
        except tk.TclError:
            return
        if not self._guard and key in ("tire_thresh", "tire_gain",
                                       "grip_thresh", "grip_gain"):
            self.mode_var.set("custom")

    def _on_brake(self):
        self.config["brake_gate"] = bool(self.brake_var.get())

    def _on_mode(self):
        mode = self.mode_var.get()
        if mode in TRIGGER_PRESETS:
            self._guard = True
            try:
                for k, v in TRIGGER_PRESETS[mode].items():
                    self.vars[k].set(v)
                    self.config[k] = v
            finally:
                self._guard = False

    def _reset(self):
        self._guard = True
        try:
            for k, v in DEFAULTS.items():
                if k == "brake_gate":
                    self.brake_var.set(v)
                else:
                    self.vars[k].set(v)
            self.config.update(DEFAULTS)
            self.mode_var.set("tarmac")
        finally:
            self._guard = False

    def _toggle(self):
        if self.engine.enabled.is_set():
            self.engine.enabled.clear()
            self.toggle_btn.config(text="START")
        else:
            self.engine.enabled.set()
            self.toggle_btn.config(text="STOP")

    def _tick(self):
        s = self.engine.status
        m = self.engine.meter
        if s["error"]:
            state = f"Error: {s['error']}"
        elif not s["game"]:
            state = "Waiting for game (start AC Rally)…"
        elif not s["pad"]:
            state = "Game found — waiting for controller…"
        elif not self.engine.enabled.is_set():
            state = "Paused (motors off) — press START to resume"
        else:
            state = "Running — driving haptics live"
        on = self.engine.enabled.is_set() and s["game"] and s["pad"]
        self.status_lbl.config(text=("● " if on else "○ ") + state)
        self.mon_lbl.config(
            text=(f"Front slip {m['fl']:+.2f} / {m['fr']:+.2f}   "
                  f"Rear slip {m['rl']:+.2f} / {m['rr']:+.2f}\n"
                  f"Triggers L {m['left']:.0%}  R {m['right']:.0%}   "
                  f"Grip low {m['low']:.0%}  high {m['high']:.0%}"))
        self.root.after(200, self._tick)

    def _close(self):
        self.engine.stop_flag.set()
        self.engine.join(timeout=2.0)
        self.root.destroy()


def main():
    root = tk.Tk()
    HapticsUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
