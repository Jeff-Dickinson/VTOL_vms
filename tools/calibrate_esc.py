#!/usr/bin/env python3
"""
ESC calibration tool.

Walks through the ESC calibration procedure:
1. Send max throttle signal
2. Power on ESCs (plug in battery)
3. Wait for confirmation tones
4. Send min throttle signal
5. Wait for confirmation tones

SAFETY: Remove propellers before running this tool!

Usage: sudo python3 tools/calibrate_esc.py
"""

from __future__ import annotations

import ctypes
import sys
import time
from pathlib import Path


def main():
    build_dir = Path("build")
    hal_path = build_dir / "libvms_hal.so"

    if not hal_path.exists():
        print(f"Error: HAL library not found at {hal_path}")
        sys.exit(1)

    hal = ctypes.CDLL(str(hal_path))

    print("=" * 60)
    print("ESC CALIBRATION TOOL")
    print("=" * 60)
    print()
    print("WARNING: REMOVE ALL PROPELLERS BEFORE PROCEEDING!")
    print()
    input("Press Enter when propellers are removed and ESCs are UNPOWERED...")

    # Init GPIO
    hal.vms_gpio_init.restype = ctypes.c_int
    if hal.vms_gpio_init() != 0:
        print("Error: GPIO init failed")
        sys.exit(1)

    # Init PCA9685
    hal.vms_pwm_init.restype = ctypes.c_int
    hal.vms_pwm_init.argtypes = [ctypes.c_int, ctypes.c_int]
    if hal.vms_pwm_init(1, 0x40) != 0:
        print("Error: PCA9685 init failed")
        hal.vms_gpio_close()
        sys.exit(1)

    hal.vms_pwm_set.restype = ctypes.c_int
    hal.vms_pwm_set.argtypes = [ctypes.c_int, ctypes.c_uint16]

    MOTOR_L_CH = 0
    MOTOR_R_CH = 1
    MAX_US = 2000
    MIN_US = 1000

    print(f"\nStep 1: Sending MAX throttle ({MAX_US}μs) to ESCs...")
    hal.vms_pwm_set(MOTOR_L_CH, MAX_US)
    hal.vms_pwm_set(MOTOR_R_CH, MAX_US)

    print("\nNow POWER ON the ESCs (connect battery).")
    print("You should hear the ESC startup tones, then a special calibration tone.")
    input("Press Enter after you hear the calibration confirmation tone...")

    print(f"\nStep 2: Sending MIN throttle ({MIN_US}μs) to ESCs...")
    hal.vms_pwm_set(MOTOR_L_CH, MIN_US)
    hal.vms_pwm_set(MOTOR_R_CH, MIN_US)

    print("You should hear confirmation tones (usually a musical scale).")
    input("Press Enter after you hear the confirmation tones...")

    print("\nCalibration complete! ESCs should now be calibrated.")
    print("The throttle range is now mapped to 1000-2000μs.")

    hal.vms_pwm_all_off()
    hal.vms_pwm_close()
    hal.vms_gpio_close()


if __name__ == "__main__":
    main()
