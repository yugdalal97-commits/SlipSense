"""Live Tkinter control panel for Assetto Corsa Rally controller haptics."""

import mmap
import threading
import time
import tkinter as tk
from tkinter import ttk

from haptics_core import (
    DEFAULTS,
    PAD_CHECK_FRAMES,
    SHM_NAME,
    SHM_SIZE,
    STALE_MUTE_SEC,
    TRIGGER_PRESETS,
    DeviceDisconnected,
    GameInputPad,
    HapticsState,
    PhysicsBlock,
    compute_haptics,
    validate_physics_layout,
)

CONFIG_REFRESH_FRAMES = 256
METER_EVERY_FRAMES = 64


class HapticsEngine(threading.Thread):
    def __init__(self, config, config_lock):
        super().__init__(daemon=True)
        self.config = config
        self.config_lock = config_lock
        self.enabled = threading.Event()
        self.enabled.set()
        self.stop_flag = threading.Event()
        self.status = {"game": False, "pad": False, "error": ""}
        self.meter = {
            "fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0,
            "left": 0.0, "right": 0.0, "low": 0.0, "high": 0.0,
        }

    def _snap_config(self):
        with self.config_lock:
            return dict(self.config)

    def run(self):
        mm = None
        physics = None
        pad = None
        try:
            validate_physics_layout()
            while mm is None and not self.stop_flag.is_set():
                try:
                    mm = mmap.mmap(-1, SHM_SIZE, tagname=SHM_NAME,
                                   access=mmap.ACCESS_WRITE)
                except (FileNotFoundError, OSError):
                    self.stop_flag.wait(0.25)
            if mm is None:
                return
            physics = PhysicsBlock.from_buffer(mm)
            self.status["game"] = True

            pad = GameInputPad()
            pad.start()
            config = self._snap_config()
            state = HapticsState()
            frame = 0
            zeroed = False
            last_packet = None
            packet_time = time.monotonic()

            while not self.stop_flag.is_set():
                if not pad.wait_for_device(self.stop_flag, retry_delay=0.25):
                    break
                self.status["pad"] = True
                state = HapticsState()
                zeroed = False
                frame = 0
                last_packet = None
                packet_time = time.monotonic()

                while not self.stop_flag.is_set():
                    now = time.monotonic()
                    if not self.enabled.is_set():
                        if not zeroed:
                            try:
                                pad.set_all(0.0, 0.0, 0.0, 0.0)
                            except DeviceDisconnected:
                                self.status["pad"] = False
                                break
                            zeroed = True
                            self.meter.update(
                                fl=0.0, fr=0.0, rl=0.0, rr=0.0,
                                left=0.0, right=0.0, low=0.0, high=0.0,
                            )
                        last_packet = physics.packet_id
                        packet_time = now
                        self.stop_flag.wait(0.05)
                        if not self.stop_flag.is_set() and not pad.connected():
                            self.status["pad"] = False
                            break
                    else:
                        zeroed = False
                        packet = physics.packet_id
                        if packet != last_packet:
                            last_packet = packet
                            packet_time = now
                        try:
                            if now - packet_time > STALE_MUTE_SEC:
                                pad.set_all(0.0, 0.0, 0.0, 0.0)
                                self.meter.update(
                                    fl=0.0, fr=0.0, rl=0.0, rr=0.0,
                                    left=0.0, right=0.0, low=0.0, high=0.0,
                                )
                            else:
                                low, high, left, right = compute_haptics(
                                    physics, config, state, now)
                                pad.set_all(low, high, left, right)
                                if frame % METER_EVERY_FRAMES == 0:
                                    self.meter.update(
                                        fl=physics.tire_slip_l,
                                        fr=physics.tire_slip_r,
                                        rl=physics.grip_loss_l,
                                        rr=physics.grip_loss_r,
                                        left=left, right=right,
                                        low=low, high=high,
                                    )
                        except DeviceDisconnected:
                            self.status["pad"] = False
                            break

                    frame += 1
                    if frame >= PAD_CHECK_FRAMES:
                        frame = 0
                        if not pad.connected():
                            self.status["pad"] = False
                            break
                    if frame % CONFIG_REFRESH_FRAMES == 0:
                        config = self._snap_config()
        except Exception as error:
            self.status["error"] = str(error)
        finally:
            if pad is not None:
                try:
                    pad.stop()
                except (OSError, RuntimeError) as error:
                    self.status["error"] = (
                        f"{self.status['error']}; cleanup: {error}"
                        if self.status["error"] else str(error))
            if physics is not None:
                del physics
            if mm is not None:
                try:
                    mm.close()
                except (BufferError, OSError) as error:
                    self.status["error"] = (
                        f"{self.status['error']}; shared-memory cleanup: {error}"
                        if self.status["error"] else str(error))
            self.status["game"] = False
            self.status["pad"] = False


class HapticsUI:
    def __init__(self, root):
        self.root = root
        root.title("Impulse Trigger Haptics")
        root.resizable(False, False)

        self.config = dict(DEFAULTS)
        self.config_lock = threading.Lock()
        self.engine = HapticsEngine(self.config, self.config_lock)
        self._guard = False

        self.mode_var = tk.StringVar(value="tarmac")
        self.vars = {
            key: tk.DoubleVar(value=DEFAULTS[key])
            for key in (
                "tire_thresh", "tire_gain", "grip_thresh", "grip_gain",
                "max_rpm", "redline_frac", "shift_pulse",
                "bump_thresh", "bump_gain", "lat_thresh", "lat_gain",
            )
        }
        self.brake_var = tk.BooleanVar(value=DEFAULTS["brake_gate"])

        for key, variable in self.vars.items():
            variable.trace_add("write", lambda *args, k=key: self._on_value(k))
        self.brake_var.trace_add("write", lambda *args: self._on_brake())

        self._build()
        self.engine.start()
        self._tick()
        root.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self):
        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        self.toggle_btn = ttk.Button(
            top, text="STOP", command=self._toggle, width=10)
        self.toggle_btn.grid(row=0, column=0, padx=(0, 10))
        self.status_lbl = ttk.Label(top, text="Starting…", width=42)
        self.status_lbl.grid(row=0, column=1, sticky="w")

        surface = ttk.LabelFrame(self.root, text="Surface mode", padding=8)
        surface.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
        for index, (value, text) in enumerate((
            ("tarmac", "Tarmac"), ("dirt", "Dirt"), ("custom", "Custom"),
        )):
            ttk.Radiobutton(
                surface, text=text, value=value, variable=self.mode_var,
                command=self._on_mode,
            ).grid(row=0, column=index, padx=12)

        triggers = ttk.LabelFrame(
            self.root, text="Impulse triggers", padding=8)
        triggers.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 6))
        self._slider(triggers, 0, "Left threshold (front lockup)",
                     "tire_thresh", 0.0, 1.0, 0.01)
        self._slider(triggers, 1, "Left gain", "tire_gain", 0.0, 10.0, 0.1)
        self._slider(triggers, 2, "Right threshold (rear slide)",
                     "grip_thresh", 0.0, 1.0, 0.01)
        self._slider(triggers, 3, "Right gain", "grip_gain", 0.0, 10.0, 0.1)
        ttk.Checkbutton(
            triggers, text="Mute slide channel while braking",
            variable=self.brake_var,
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

        grip = ttk.LabelFrame(
            self.root, text="Grip motors (events only)", padding=8)
        grip.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 6))
        self._slider(grip, 0, "Max RPM (redline = 100%)",
                     "max_rpm", 4000, 12000, 100)
        self._slider(grip, 1, "Redline warning from",
                     "redline_frac", 0.80, 0.99, 0.005)
        self._slider(grip, 2, "Shift thump (sec)",
                     "shift_pulse", 0.0, 0.5, 0.01)
        self._slider(grip, 3, "Landing threshold (vert G)",
                     "bump_thresh", 0.0, 2.0, 0.05)
        self._slider(grip, 4, "Landing gain",
                     "bump_gain", 0.0, 5.0, 0.1)
        self._slider(grip, 5, "Cornering threshold (lat G)",
                     "lat_thresh", 0.0, 2.0, 0.05)
        self._slider(grip, 6, "Cornering gain",
                     "lat_gain", 0.0, 5.0, 0.1)

        monitor = ttk.LabelFrame(self.root, text="Live", padding=8)
        monitor.grid(row=4, column=0, sticky="ew", padx=10, pady=(0, 6))
        self.mon_lbl = ttk.Label(
            monitor, text="Waiting for game…", font=("TkDefaultFont", 9))
        self.mon_lbl.grid(row=0, column=0, sticky="w")

        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bottom.grid(row=5, column=0, sticky="ew")
        ttk.Button(
            bottom, text="Reset to defaults", command=self._reset,
        ).grid(row=0, column=0)
        ttk.Label(
            bottom, text="Values apply live — no restart needed.",
            foreground="gray",
        ).grid(row=0, column=1, padx=(12, 0))

    def _slider(self, parent, row, label, key, low, high, resolution):
        ttk.Label(parent, text=label, width=30).grid(
            row=row, column=0, sticky="w")
        tk.Scale(
            parent, from_=low, to=high, resolution=resolution,
            orient="horizontal", length=230, showvalue=True,
            variable=self.vars[key],
        ).grid(row=row, column=1, sticky="ew")

    def _on_value(self, key):
        try:
            value = self.vars[key].get()
        except tk.TclError:
            return
        with self.config_lock:
            self.config[key] = value
        if not self._guard and key in (
                "tire_thresh", "tire_gain", "grip_thresh", "grip_gain"):
            self.mode_var.set("custom")

    def _on_brake(self):
        with self.config_lock:
            self.config["brake_gate"] = bool(self.brake_var.get())

    def _on_mode(self):
        mode = self.mode_var.get()
        if mode in TRIGGER_PRESETS:
            self._guard = True
            try:
                with self.config_lock:
                    self.config.update(TRIGGER_PRESETS[mode])
                for key, value in TRIGGER_PRESETS[mode].items():
                    self.vars[key].set(value)
            finally:
                self._guard = False

    def _reset(self):
        self._guard = True
        try:
            with self.config_lock:
                self.config.update(DEFAULTS)
            for key, value in DEFAULTS.items():
                if key == "brake_gate":
                    self.brake_var.set(value)
                elif key in self.vars:
                    self.vars[key].set(value)
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
        status = self.engine.status
        meter = self.engine.meter
        if status["error"]:
            state = f"Error: {status['error']}"
        elif not status["game"]:
            state = "Waiting for game (start AC Rally)…"
        elif not status["pad"]:
            state = "Game found — waiting for controller…"
        elif not self.engine.enabled.is_set():
            state = "Paused (motors off) — press START to resume"
        else:
            state = "Running — driving haptics live"
        active = (
            self.engine.enabled.is_set() and status["game"] and status["pad"])
        self.status_lbl.config(text=("● " if active else "○ ") + state)
        self.mon_lbl.config(
            text=(f"Front slip {meter['fl']:+.2f} / {meter['fr']:+.2f}   "
                  f"Rear slip {meter['rl']:+.2f} / {meter['rr']:+.2f}\n"
                  f"Triggers L {meter['left']:.0%}  R {meter['right']:.0%}   "
                  f"Grip low {meter['low']:.0%}  high {meter['high']:.0%}"))
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
