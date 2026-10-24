import ctypes
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import haptics_core
from haptics_core import (
    DEFAULTS,
    SHM_SIZE,
    TRIGGER_PRESETS,
    HapticsState,
    GameInputPad,
    PhysicsBlock,
    compute_haptics,
    validate_physics_layout,
)


def sample(**overrides):
    values = {
        "tire_slip_l": 0.0,
        "tire_slip_r": 0.0,
        "grip_loss_l": 0.0,
        "grip_loss_r": 0.0,
        "brake": 0.0,
        "gear": 2,
        "rpm": 0,
        "vert_g": 0.0,
        "g_lat": 0.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class HapticsCalculationTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(DEFAULTS)
        self.state = HapticsState()

    def test_physics_fields_match_shared_memory_offsets(self):
        validate_physics_layout()
        self.assertEqual(ctypes.sizeof(PhysicsBlock), SHM_SIZE)
        self.assertEqual(PhysicsBlock.tire_slip_l.offset, 56)
        self.assertEqual(PhysicsBlock.grip_loss_r.offset, 68)

    def test_uses_loudest_absolute_wheel_slip_and_clamps(self):
        levels = compute_haptics(
            sample(tire_slip_l=-0.375, tire_slip_r=0.2,
                   grip_loss_l=0.1, grip_loss_r=-0.2),
            self.config, self.state, 1.0,
        )
        self.assertAlmostEqual(levels[2], 0.5)
        self.assertAlmostEqual(levels[3], 0.96)

        saturated = compute_haptics(
            sample(tire_slip_l=10.0, grip_loss_l=10.0),
            self.config, self.state, 2.0,
        )
        self.assertEqual(saturated[2:], (1.0, 1.0))

    def test_brake_gate_suppresses_rear_trigger_and_can_be_disabled(self):
        braking = sample(grip_loss_l=0.2, brake=0.30)
        self.assertEqual(
            compute_haptics(braking, self.config, self.state, 1.0)[3], 0.0)
        self.config["brake_gate"] = False
        self.assertAlmostEqual(
            compute_haptics(braking, self.config, self.state, 2.0)[3], 0.96)

    def test_redline_shift_landing_and_cornering_outputs(self):
        first = compute_haptics(
            sample(rpm=8000, vert_g=0.95), self.config, self.state, 1.0)
        self.assertAlmostEqual(first[0], 1.0)
        self.assertEqual(first[1], 1.0)

        shifted = compute_haptics(
            sample(gear=3), self.config, self.state, 1.1)
        self.assertEqual(shifted[1], 1.0)
        after_pulse = compute_haptics(
            sample(gear=3, g_lat=-1.3), self.config, self.state, 1.26)
        self.assertEqual(after_pulse[1], 0.0)
        self.assertAlmostEqual(after_pulse[0], 1.0)

    def test_console_and_ui_share_preset_values(self):
        for preset in TRIGGER_PRESETS.values():
            config = dict(DEFAULTS)
            config.update(preset)
            self.assertEqual(config["tire_thresh"], preset["tire_thresh"])
            self.assertEqual(config["grip_gain"], preset["grip_gain"])


class GameInputLifecycleTests(unittest.TestCase):
    def make_pad(self):
        pad = GameInputPad.__new__(GameInputPad)
        pad._device_lock = threading.Lock()
        pad._device_event = threading.Event()
        pad._device = 101
        return pad

    def test_disconnect_callback_clears_device_and_wakes_waiters(self):
        pad = self.make_pad()
        pad._device_event.set()
        released = []
        with patch.object(haptics_core, "_release", side_effect=released.append):
            pad._on_device_changed(101, 0, 1)

        self.assertIsNone(pad._device)
        self.assertFalse(pad._device_event.is_set())
        self.assertEqual(released, [101])

    def test_connect_callback_replaces_old_device_and_signals_waiters(self):
        pad = self.make_pad()
        added = []
        released = []
        with patch.object(haptics_core, "_addref", side_effect=added.append), \
                patch.object(haptics_core, "_release", side_effect=released.append):
            pad._on_device_changed(202, 1, 0)

        self.assertEqual(pad._device, 202)
        self.assertTrue(pad._device_event.is_set())
        self.assertEqual(added, [202])
        self.assertEqual(released, [101])


if __name__ == "__main__":
    unittest.main()
