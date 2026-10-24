"""Shared telemetry layout, GameInput lifecycle, and haptics calculations."""

import ctypes
import threading
from dataclasses import dataclass


SHM_NAME = r"Local\acpmf_physics"
SHM_SIZE = 128

BRAKE_OFFSET = 8
GEAR_OFFSET = 16
RPM_OFFSET = 20
LAT_G_OFFSET = 44
VERT_G_OFFSET = 52
TIRE_SLIP_OFFSET_L = 56
TIRE_SLIP_OFFSET_R = 60
GRIP_LOSS_OFFSET_L = 64
GRIP_LOSS_OFFSET_R = 68

STALE_MUTE_SEC = 0.5
PAD_CHECK_FRAMES = 1024
SETUP_RETRY_DELAY = 1.0

DEFAULTS = {
    "tire_thresh": 0.25,
    "tire_gain": 4.0,
    "grip_thresh": 0.08,
    "grip_gain": 8.0,
    "tire_slip_max": 1.0,
    "grip_loss_max": 1.0,
    "brake_gate": True,
    "max_rpm": 8000.0,
    "rpm_gain": 1.0,
    "redline_frac": 0.92,
    "shift_pulse": 0.15,
    "bump_thresh": 0.45,
    "bump_gain": 2.0,
    "lat_thresh": 0.80,
    "lat_gain": 2.0,
}

TRIGGER_PRESETS = {
    "tarmac": {
        "tire_thresh": 0.25, "tire_gain": 4.0,
        "grip_thresh": 0.08, "grip_gain": 8.0,
    },
    "dirt": {
        "tire_thresh": 0.35, "tire_gain": 3.0,
        "grip_thresh": 0.35, "grip_gain": 3.0,
    },
}

_FLOAT_SIZE = ctypes.sizeof(ctypes.c_float)
_INT_SIZE = ctypes.sizeof(ctypes.c_int)


class PhysicsBlock(ctypes.Structure):
    """Packed view of the Assetto Corsa Rally physics shared-memory block."""

    _pack_ = 1
    _fields_ = [
        ("packet_id", ctypes.c_int),
        ("_pad_gas", ctypes.c_byte * (BRAKE_OFFSET - _INT_SIZE)),
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


def validate_physics_layout():
    expected = {
        "packet_id": 0,
        "brake": BRAKE_OFFSET,
        "gear": GEAR_OFFSET,
        "rpm": RPM_OFFSET,
        "g_lat": LAT_G_OFFSET,
        "vert_g": VERT_G_OFFSET,
        "tire_slip_l": TIRE_SLIP_OFFSET_L,
        "tire_slip_r": TIRE_SLIP_OFFSET_R,
        "grip_loss_l": GRIP_LOSS_OFFSET_L,
        "grip_loss_r": GRIP_LOSS_OFFSET_R,
    }
    for field, offset in expected.items():
        actual = getattr(PhysicsBlock, field).offset
        if actual != offset:
            raise RuntimeError(f"Physics field {field} is at {actual}; expected {offset}")
    required = max(expected.values()) + _FLOAT_SIZE
    if ctypes.sizeof(PhysicsBlock) != SHM_SIZE or SHM_SIZE < required:
        raise RuntimeError("PhysicsBlock size does not cover the configured telemetry offsets")


@dataclass
class HapticsState:
    last_gear: int = None
    pulse_end: float = 0.0


def _clamp(value):
    return max(0.0, min(1.0, value))


def compute_haptics(physics, config, state, now):
    """Return (low motor, high motor, left trigger, right trigger) intensities."""
    front_slip = max(abs(physics.tire_slip_l), abs(physics.tire_slip_r))
    rear_slip = max(abs(physics.grip_loss_l), abs(physics.grip_loss_r))

    tire_max = config.get("tire_slip_max", 1.0) or 1.0
    grip_max = config.get("grip_loss_max", 1.0) or 1.0
    left = ((front_slip - config["tire_thresh"]) * config["tire_gain"]
            / tire_max)
    right = ((rear_slip - config["grip_thresh"]) * config["grip_gain"]
             / grip_max)
    if config.get("brake_gate", True) and physics.brake > 0.05:
        gate = 1.0 - (physics.brake - 0.05) / 0.25
        right *= max(0.0, gate)
    left = _clamp(left)
    right = _clamp(right)

    gear = physics.gear
    if state.last_gear is None:
        state.last_gear = gear
    elif gear != state.last_gear:
        state.last_gear = gear
        state.pulse_end = now + config["shift_pulse"]

    max_rpm = config["max_rpm"] or 8000.0
    redline = config["redline_frac"]
    redline_span = 1.0 - redline if redline < 1.0 else 0.08
    rpm_fraction = physics.rpm * (config.get("rpm_gain", 1.0) / max_rpm)
    high = _clamp((rpm_fraction - redline) / redline_span) if rpm_fraction > redline else 0.0
    if now < state.pulse_end:
        high = 1.0

    bump = max(0.0, physics.vert_g - config["bump_thresh"]) * config["bump_gain"]
    lateral = (abs(physics.g_lat) - config["lat_thresh"]) * config["lat_gain"]
    low = max(_clamp(bump), _clamp(lateral))
    return low, high, left, right


class DeviceDisconnected(RuntimeError):
    """Raised when a haptics write is attempted without a connected pad."""


class _GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", ctypes.c_uint32),
        ("Data2", ctypes.c_uint16),
        ("Data3", ctypes.c_uint16),
        ("Data4", ctypes.c_ubyte * 8),
    ]


class RumbleParams(ctypes.Structure):
    _fields_ = [
        ("lowFrequency", ctypes.c_float),
        ("highFrequency", ctypes.c_float),
        ("leftTrigger", ctypes.c_float),
        ("rightTrigger", ctypes.c_float),
    ]


_IID_IGAMEINPUT = _GUID(
    0x20EFC1C7, 0x5D9A, 0x43BA,
    (ctypes.c_ubyte * 8)(0xB2, 0x6F, 0xB8, 0x07, 0xFA, 0x48, 0x60, 0x9C),
)
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
    """Own GameInput and a thread-safe reference to the connected gamepad."""

    def __init__(self):
        self.abi = None
        self.gi = None
        self.lib = None
        self.token = ctypes.c_uint64(0)
        self._device = None
        self._device_lock = threading.Lock()
        self._device_event = threading.Event()
        self._callback = None
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
            raise RuntimeError("GameInput initialization failed")
        self.lib, self.gi, self.abi = lib, out.value, "v0"

    def _idx(self, name):
        return _IDX[self.abi][name]

    def _clear_device(self, expected=None):
        with self._device_lock:
            device = self._device
            if device is None or (expected is not None and device != expected):
                return
            self._device = None
            self._device_event.clear()
        _release(device)

    def _acquire_device(self):
        with self._device_lock:
            device = self._device
            if device is not None:
                _addref(device)
            return device

    def _on_device_changed(self, device, current, previous):
        if device and current & _STATUS_CONNECTED:
            with self._device_lock:
                if self._device != device:
                    _addref(device)
                    old, self._device = self._device, device
                else:
                    old = None
                self._device_event.set()
            if old:
                _release(old)
        elif device and previous & _STATUS_CONNECTED:
            self._clear_device(device)

    def start(self):
        callback_type = ctypes.WINFUNCTYPE(
            None, ctypes.c_uint64, ctypes.c_void_p, ctypes.c_void_p,
            ctypes.c_uint64, ctypes.c_int, ctypes.c_int,
        )

        def on_device(token, context, device, timestamp, current, previous):
            self._on_device_changed(device, current, previous)

        self._callback = callback_type(on_device)
        register = _vfunc(
            self.gi, self._idx("reg"), ctypes.c_long,
            [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_int,
             ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        result = register(
            self.gi, None, _KIND_GAMEPAD, _STATUS_CONNECTED, _ENUM_ASYNC,
            None, ctypes.cast(self._callback, ctypes.c_void_p),
            ctypes.byref(self.token),
        )
        if result != 0:
            raise RuntimeError(
                f"RegisterDeviceCallback failed: 0x{result & 0xFFFFFFFF:08X}")

    def connected(self):
        device = self._acquire_device()
        if device is None:
            return False
        try:
            get_status = _vfunc(device, self._idx("status"), ctypes.c_int, [])
            if get_status(device) & _STATUS_CONNECTED:
                return True
            self._clear_device(device)
            return False
        finally:
            _release(device)

    def wait_for_device(self, stop_event, retry_delay=SETUP_RETRY_DELAY):
        while not stop_event.is_set():
            if self.connected():
                return True
            self._device_event.wait(retry_delay)
        return False

    def set_all(self, low, high, left, right):
        device = self._acquire_device()
        if device is None:
            raise DeviceDisconnected("No connected GameInput controller")
        try:
            self._rumble.lowFrequency = low
            self._rumble.highFrequency = high
            self._rumble.leftTrigger = left
            self._rumble.rightTrigger = right
            _vfunc(device, self._idx("rumble"), None, [ctypes.c_void_p])(
                device, ctypes.byref(self._rumble))
        finally:
            _release(device)

    def stop(self):
        errors = []
        device = self._device
        if device is not None:
            try:
                self.set_all(0.0, 0.0, 0.0, 0.0)
            except DeviceDisconnected:
                pass
            except OSError as error:
                errors.append(error)
        try:
            if self.abi == "v3":
                unregister = _vfunc(
                    self.gi, self._idx("unreg"), ctypes.c_bool, [ctypes.c_uint64])
                unregister(self.gi, self.token.value)
            else:
                unregister = _vfunc(
                    self.gi, self._idx("unreg"), ctypes.c_bool,
                    [ctypes.c_uint64, ctypes.c_uint64])
                unregister(self.gi, self.token.value, 1000000)
        except OSError as error:
            errors.append(error)
        try:
            self._clear_device()
        except OSError as error:
            errors.append(error)
        try:
            _release(self.gi)
        except OSError as error:
            errors.append(error)
        self.gi = None
        if errors:
            raise RuntimeError("GameInput cleanup failed: " + "; ".join(map(str, errors)))
