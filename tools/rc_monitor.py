#!/usr/bin/env python3
"""
RC receiver monitor tool.

Displays live RC channel values from the FlySky FS-R9B receiver.
Used to verify receiver wiring, check channel assignments, and test
deadbands/endpoints.

Usage: sudo python3 tools/rc_monitor.py
"""

from __future__ import annotations

import ctypes
import sys
import time
from pathlib import Path

import yaml


def main():
    build_dir = Path("build")
    hal_path = build_dir / "libvms_hal.so"
    config_path = Path("config/vehicle.yaml")

    if not hal_path.exists():
        print(f"Error: HAL library not found at {hal_path}")
        sys.exit(1)

    hal = ctypes.CDLL(str(hal_path))

    # Load pin config
    with open(config_path) as f:
        vehicle = yaml.safe_load(f)

    pin_map = vehicle["rc_gpio_pins"]
    pin_keys = sorted(pin_map.keys())
    pins = [pin_map[k] for k in pin_keys]
    num_pins = len(pins)

    # Init GPIO
    hal.vms_gpio_init.restype = ctypes.c_int
    if hal.vms_gpio_init() != 0:
        print("Error: GPIO init failed")
        sys.exit(1)

    # Init RC input
    PinArray = ctypes.c_int * num_pins
    c_pins = PinArray(*pins)

    hal.vms_rc_init.restype = ctypes.c_int
    hal.vms_rc_init.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.c_int]
    if hal.vms_rc_init(c_pins, num_pins) != 0:
        print("Error: RC input init failed")
        hal.vms_gpio_close()
        sys.exit(1)

    hal.vms_rc_read.restype = ctypes.c_int
    hal.vms_rc_read.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_int]
    hal.vms_rc_ms_since_last.restype = ctypes.c_uint32

    channel_names = ["Roll", "Pitch", "Throt", "Yaw", "Mode", "Tilt", "Flap", "Kill"]

    print("RC Monitor — Press Ctrl+C to stop\n")
    header = "  ".join(f"{name:>6s}" for name in channel_names[:num_pins])
    print(f"  {header}   Signal")
    print("-" * (8 * num_pins + 15))

    ChannelArray = ctypes.c_uint16 * num_pins

    try:
        while True:
            channels = ChannelArray()
            valid = hal.vms_rc_read(channels, num_pins)
            ms_since = hal.vms_rc_ms_since_last()

            values = "  ".join(f"{channels[i]:6d}" for i in range(num_pins))
            status = "OK" if valid else f"LOST ({ms_since}ms)"
            print(f"\r  {values}   {status:20s}", end="", flush=True)

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        hal.vms_rc_close()
        hal.vms_gpio_close()


if __name__ == "__main__":
    main()
