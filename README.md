# SlipSense

SlipSense reads Assetto Corsa Rally physics from Windows named shared memory
and sends controller haptics through Microsoft's native GameInput API. The UI
and console runner share the same telemetry layout, GameInput device lifecycle,
presets, and signal calculations in `haptics_core.py`.

## Requirements

- Windows 10 or later and Python 3.10+
- Assetto Corsa Rally exposing the `Local\acpmf_physics` mapping
- A connected GameInput-compatible controller
- GameInput available from Windows or the installed `GameInputRedist.dll`

The haptics tools use Python's standard library; no package installation is
needed. `buzz_test.py` is an optional Windows Gaming Input test and requires
the `winsdk` Python package. `xinput_buzz_test.py` tests the controller's main
motors through the Windows XInput DLLs.

## Run

Start the game and controller, then launch either the control panel:

```powershell
python haptics_ui.py
```

Or run the console haptics loop with the default tarmac tune, or select dirt:

```powershell
python telemetry_impulse_triggers.py
python telemetry_impulse_triggers.py dirt
```

The console runner waits for the game's shared-memory mapping and a
GameInput-connected controller. Press Ctrl+C to stop. The UI exposes live
trigger thresholds/gains, brake gating, engine/shift feedback, and landing/
cornering feedback.

## Telemetry layout

`haptics_core.py` defines the current 128-byte mapping for the Assetto Corsa
Rally physics block. The packet counter is at byte 0; brake at 8; gear at 16;
RPM at 20; lateral G at 44; vertical G at 52; and wheel slip values at
56/60/64/68 (front-left, front-right, rear-left, rear-right). Adjust those
constants and `SHM_NAME` together if targeting a different simulator, then run
the tests to verify the packed structure.

## Verify

Run the platform-independent signal and telemetry-layout tests:

```powershell
python -m unittest discover -v
```
