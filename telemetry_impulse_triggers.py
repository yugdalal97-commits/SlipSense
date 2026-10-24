"""Console telemetry-to-haptics runner for Assetto Corsa Rally."""

import argparse
import mmap
import threading
import time

from haptics_core import (
    DEFAULTS,
    PAD_CHECK_FRAMES,
    SETUP_RETRY_DELAY,
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

SURFACE_MODE = "tarmac"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive Xbox controller haptics from shared-memory telemetry.")
    parser.add_argument(
        "surface", nargs="?", default=SURFACE_MODE,
        choices=tuple(TRIGGER_PRESETS),
        help="trigger preset to use (default: %(default)s)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    validate_physics_layout()

    print(f"Waiting for shared memory '{SHM_NAME}' ... (start the game)", flush=True)
    mm = None
    pad = None
    physics = None
    try:
        while mm is None:
            try:
                mm = mmap.mmap(-1, SHM_SIZE, tagname=SHM_NAME,
                               access=mmap.ACCESS_WRITE)
            except (FileNotFoundError, OSError):
                time.sleep(SETUP_RETRY_DELAY)
        print(f"Attached to '{SHM_NAME}' ({SHM_SIZE} bytes).", flush=True)

        pad = GameInputPad()
        pad.start()
        config = dict(DEFAULTS)
        config.update(TRIGGER_PRESETS[args.surface])
        print(f"Waiting for a GameInput controller (surface: {args.surface})...",
              flush=True)
        physics = PhysicsBlock.from_buffer(mm)

        while True:
            if not pad.wait_for_device(threading.Event()):
                break
            print("Controller connected. Forwarding telemetry; Ctrl+C to stop.",
                  flush=True)
            state = HapticsState()
            last_packet = None
            packet_time = time.monotonic()
            frame = 0

            while True:
                now = time.monotonic()
                packet = physics.packet_id
                if packet != last_packet:
                    last_packet = packet
                    packet_time = now

                try:
                    if now - packet_time > STALE_MUTE_SEC:
                        pad.set_all(0.0, 0.0, 0.0, 0.0)
                    else:
                        pad.set_all(*compute_haptics(physics, config, state, now))
                except DeviceDisconnected:
                    print("Controller disconnected; waiting to reconnect...",
                          flush=True)
                    break

                frame += 1
                if frame >= max(1, PAD_CHECK_FRAMES):
                    frame = 0
                    if not pad.connected():
                        print("Controller disconnected; waiting to reconnect...",
                              flush=True)
                        break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if pad is not None:
                pad.stop()
        finally:
            if physics is not None:
                del physics
            if mm is not None:
                mm.close()
        print("\nStopped. Triggers zeroed, shared memory released.", flush=True)


if __name__ == "__main__":
    main()
