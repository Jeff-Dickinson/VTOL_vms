#!/usr/bin/env python3
"""
IMU calibration and verification tool.

Reads BNO085 quaternion/Euler output and displays it in real-time.
Used to verify IMU is working correctly and check calibration status.

Usage: sudo python3 tools/calibrate_imu.py
"""

from __future__ import annotations

import ctypes
import sys
import time
from pathlib import Path


def main():
    build_dir = Path("build")
    hal_path = build_dir / "libvms_hal.so"
    control_path = build_dir / "libvms_control.so"

    if not hal_path.exists() or not control_path.exists():
        print(f"Error: C libraries not found in {build_dir}/")
        print("Run: cd build && cmake .. && make")
        sys.exit(1)

    hal = ctypes.CDLL(str(hal_path))
    control = ctypes.CDLL(str(control_path))

    # Import shared state accessor
    from vms.ipc import SharedMemoryIPC
    ipc = SharedMemoryIPC(control)
    ipc.init()

    # Init GPIO (pigpio)
    hal.vms_gpio_init.restype = ctypes.c_int
    if hal.vms_gpio_init() != 0:
        print("Error: GPIO init failed")
        sys.exit(1)

    # Init BNO085
    hal.vms_bno085_init.restype = ctypes.c_int
    hal.vms_bno085_init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
    if hal.vms_bno085_init(1, 0x4A, 100) != 0:
        print("Error: BNO085 init failed. Check I2C connection.")
        hal.vms_gpio_close()
        sys.exit(1)

    print("BNO085 initialized. Reading at 100Hz. Press Ctrl+C to stop.\n")
    print(f"{'Roll':>8s} {'Pitch':>8s} {'Yaw':>8s}  |  {'Qw':>7s} {'Qx':>7s} {'Qy':>7s} {'Qz':>7s}")
    print("-" * 70)

    class ImuData(ctypes.Structure):
        _fields_ = [
            ("w", ctypes.c_float), ("x", ctypes.c_float),
            ("y", ctypes.c_float), ("z", ctypes.c_float),
            ("gyro_x", ctypes.c_float), ("gyro_y", ctypes.c_float),
            ("gyro_z", ctypes.c_float),
            ("accuracy", ctypes.c_uint8), ("valid", ctypes.c_uint8),
        ]

    hal.vms_bno085_read.restype = ctypes.c_int
    hal.vms_bno085_read.argtypes = [ctypes.POINTER(ImuData)]

    # Import attitude conversion
    control.vms_quat_to_euler.restype = None  # We'll do it in Python instead
    import math

    try:
        while True:
            data = ImuData()
            if hal.vms_bno085_read(ctypes.byref(data)) == 0:
                # Quaternion to Euler (simplified)
                w, x, y, z = data.w, data.x, data.y, data.z
                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                sinp = 2 * (w * y - z * x)
                sinp = max(-1, min(1, sinp))
                pitch = math.degrees(math.asin(sinp))

                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                print(f"\r{roll:8.2f} {pitch:8.2f} {yaw:8.2f}  |  "
                      f"{w:7.4f} {x:7.4f} {y:7.4f} {z:7.4f}  "
                      f"acc={data.accuracy}", end="", flush=True)

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        hal.vms_bno085_close()
        hal.vms_gpio_close()


if __name__ == "__main__":
    main()
