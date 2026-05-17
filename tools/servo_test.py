#!/usr/bin/env python3
"""
Servo sweep test tool.

Sweeps each servo through its full range to verify wiring and direction.
Also allows manual positioning of individual servos.

Usage: sudo python3 tools/servo_test.py [--channel CH] [--sweep]
"""

from __future__ import annotations

import argparse
import ctypes
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Servo test tool")
    parser.add_argument("--channel", "-c", type=int, default=-1,
                        help="Test a specific PCA9685 channel (0-15). Default: all servos.")
    parser.add_argument("--sweep", action="store_true",
                        help="Continuously sweep servos back and forth")
    parser.add_argument("--position", "-p", type=int, default=None,
                        help="Set servo to specific position (μs, 1000-2000)")
    args = parser.parse_args()

    build_dir = Path("build")
    hal_path = build_dir / "libvms_hal.so"

    if not hal_path.exists():
        print(f"Error: HAL library not found at {hal_path}")
        sys.exit(1)

    hal = ctypes.CDLL(str(hal_path))

    hal.vms_gpio_init.restype = ctypes.c_int
    if hal.vms_gpio_init() != 0:
        print("Error: GPIO init failed")
        sys.exit(1)

    hal.vms_pwm_init.restype = ctypes.c_int
    hal.vms_pwm_init.argtypes = [ctypes.c_int, ctypes.c_int]
    if hal.vms_pwm_init(1, 0x40) != 0:
        print("Error: PCA9685 init failed")
        hal.vms_gpio_close()
        sys.exit(1)

    hal.vms_pwm_set.restype = ctypes.c_int
    hal.vms_pwm_set.argtypes = [ctypes.c_int, ctypes.c_uint16]

    # Servo channel names for display
    servo_names = {
        0: "Motor Left (ESC)", 1: "Motor Right (ESC)",
        2: "Tilt Left", 3: "Tilt Right",
        4: "Aileron Left", 5: "Aileron Right",
        6: "Flap Left", 7: "Flap Right",
        8: "Rudder",
    }

    # Only test servo channels (skip ESCs: 0, 1)
    servo_channels = [2, 3, 4, 5, 6, 7, 8]

    if args.channel >= 0:
        channels = [args.channel]
    else:
        channels = servo_channels

    try:
        if args.position is not None:
            pos = max(1000, min(2000, args.position))
            for ch in channels:
                name = servo_names.get(ch, f"Channel {ch}")
                print(f"Setting {name} (CH{ch}) to {pos}μs")
                hal.vms_pwm_set(ch, pos)
            print("Press Ctrl+C to stop and center servos.")
            while True:
                time.sleep(1)

        elif args.sweep:
            print("Sweeping servos. Press Ctrl+C to stop.")
            while True:
                # Sweep min → max
                for us in range(1000, 2001, 10):
                    for ch in channels:
                        hal.vms_pwm_set(ch, us)
                    time.sleep(0.02)
                # Sweep max → min
                for us in range(2000, 999, -10):
                    for ch in channels:
                        hal.vms_pwm_set(ch, us)
                    time.sleep(0.02)

        else:
            # Single sweep test
            for ch in channels:
                name = servo_names.get(ch, f"Channel {ch}")
                print(f"Testing {name} (CH{ch})...")

                print("  → Center (1500μs)")
                hal.vms_pwm_set(ch, 1500)
                time.sleep(0.5)

                print("  → Min (1000μs)")
                hal.vms_pwm_set(ch, 1000)
                time.sleep(0.5)

                print("  → Max (2000μs)")
                hal.vms_pwm_set(ch, 2000)
                time.sleep(0.5)

                print("  → Center (1500μs)")
                hal.vms_pwm_set(ch, 1500)
                time.sleep(0.3)

            print("\nAll servos tested.")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Center all servos
        for ch in channels:
            hal.vms_pwm_set(ch, 1500)
        time.sleep(0.1)
        hal.vms_pwm_all_off()
        hal.vms_pwm_close()
        hal.vms_gpio_close()


if __name__ == "__main__":
    main()
