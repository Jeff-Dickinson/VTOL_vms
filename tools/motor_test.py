#!/usr/bin/env python3
"""
Motor test tool.

SAFETY: PROPELLERS MUST BE REMOVED!

Sends controlled throttle commands to verify ESC/motor operation.
Ramps up slowly to a configurable maximum, then back down.

Usage: sudo python3 tools/motor_test.py [--max-throttle 0.2] [--motor left|right|both]
"""

from __future__ import annotations

import argparse
import ctypes
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Motor test (PROPS OFF ONLY)")
    parser.add_argument("--max-throttle", type=float, default=0.15,
                        help="Maximum throttle (0.0-1.0). Default: 0.15 (15%%)")
    parser.add_argument("--motor", choices=["left", "right", "both"], default="both",
                        help="Which motor to test")
    parser.add_argument("--ramp-time", type=float, default=3.0,
                        help="Seconds to ramp to max throttle")
    args = parser.parse_args()

    if args.max_throttle > 0.3:
        print("ERROR: max-throttle > 30%% is too dangerous for bench testing!")
        print("       Use a lower value or test on a restrained vehicle.")
        sys.exit(1)

    build_dir = Path("build")
    hal_path = build_dir / "libvms_hal.so"

    if not hal_path.exists():
        print(f"Error: HAL library not found at {hal_path}")
        sys.exit(1)

    hal = ctypes.CDLL(str(hal_path))

    print("=" * 60)
    print("MOTOR TEST TOOL")
    print("=" * 60)
    print()
    print("!!! PROPELLERS MUST BE REMOVED !!!")
    print(f"Motor:         {args.motor}")
    print(f"Max throttle:  {args.max_throttle * 100:.0f}%")
    print(f"Ramp time:     {args.ramp_time:.1f}s")
    print()
    confirm = input("Type 'YES' to confirm props are off and proceed: ")
    if confirm != "YES":
        print("Aborted.")
        sys.exit(0)

    # Init HAL
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

    MOTOR_L = 0
    MOTOR_R = 1
    ESC_MIN = 1000
    ESC_MAX = 2000

    motors = []
    if args.motor in ("left", "both"):
        motors.append(MOTOR_L)
    if args.motor in ("right", "both"):
        motors.append(MOTOR_R)

    def set_throttle(throttle: float):
        us = int(ESC_MIN + throttle * (ESC_MAX - ESC_MIN))
        us = max(ESC_MIN, min(ESC_MAX, us))
        for ch in motors:
            hal.vms_pwm_set(ch, us)

    try:
        # Arm ESCs
        print("\nArming ESCs (sending min throttle)...")
        set_throttle(0.0)
        time.sleep(2.0)

        # Ramp up
        print(f"Ramping up to {args.max_throttle * 100:.0f}%...")
        steps = int(args.ramp_time / 0.02)
        for i in range(steps + 1):
            t = (i / steps) * args.max_throttle
            set_throttle(t)
            print(f"\r  Throttle: {t * 100:5.1f}%", end="", flush=True)
            time.sleep(0.02)

        # Hold
        print(f"\n  Holding at {args.max_throttle * 100:.0f}% for 2 seconds...")
        time.sleep(2.0)

        # Ramp down
        print("Ramping down...")
        for i in range(steps, -1, -1):
            t = (i / steps) * args.max_throttle
            set_throttle(t)
            print(f"\r  Throttle: {t * 100:5.1f}%", end="", flush=True)
            time.sleep(0.02)

        print("\nTest complete.")

    except KeyboardInterrupt:
        print("\n\nEMERGENCY STOP!")
    finally:
        set_throttle(0.0)
        time.sleep(0.1)
        hal.vms_pwm_all_off()
        hal.vms_pwm_close()
        hal.vms_gpio_close()
        print("Motors stopped.")


if __name__ == "__main__":
    main()
