"""
VMS main entry point.

Boot sequence:
1. Load configuration (vehicle, PID, channel map)
2. Initialize shared memory and load C libraries
3. Push initial PID gains to shared memory
4. Start the C real-time control loop thread
5. Run the 50Hz Python orchestration loop:
   - Read shared state
   - Update flight mode state machine
   - Run safety checks
   - Write commands back to shared state
   - Log telemetry
6. On shutdown: stop control loop, close telemetry
"""

from __future__ import annotations

import argparse
import ctypes
import logging
import signal
import sys
import time
from pathlib import Path

from vms.config import load_vehicle_config, load_pid_config, load_channel_map
from vms.flight_mode import FlightMode, FlightModeManager
from vms.ipc import SharedMemoryIPC
from vms.safety import SafetyMonitor
from vms.telemetry import TelemetryLogger

log = logging.getLogger("vms")

PYTHON_LOOP_HZ = 50
PYTHON_LOOP_PERIOD = 1.0 / PYTHON_LOOP_HZ

g_shutdown = False


def _signal_handler(signum, frame):
    global g_shutdown
    if signum == signal.SIGINT or signum == signal.SIGTERM:
        log.info("Shutdown signal received")
        g_shutdown = True


def _load_c_libraries(build_dir: Path) -> tuple[ctypes.CDLL, ctypes.CDLL]:
    """Load libvms_hal.so and libvms_control.so."""
    hal_path = build_dir / "libvms_hal.so"
    control_path = build_dir / "libvms_control.so"

    if not hal_path.exists():
        raise FileNotFoundError(f"HAL library not found: {hal_path}")
    if not control_path.exists():
        raise FileNotFoundError(f"Control library not found: {control_path}")

    hal_lib = ctypes.CDLL(str(hal_path))
    control_lib = ctypes.CDLL(str(control_path))
    return hal_lib, control_lib


def _build_hal_config(vehicle_cfg, control_lib):
    """Build the C HAL config struct from Python config."""
    # Import the struct type from control_loop.h via ctypes

    class HalConfig(ctypes.Structure):
        _fields_ = [
            ("i2c_bus", ctypes.c_int),
            ("bno085_addr", ctypes.c_int),
            ("pca9685_addr", ctypes.c_int),
            ("rc_pins", ctypes.c_int * 8),
            ("rc_pin_count", ctypes.c_int),
            ("pwm_channels", ctypes.c_int * 9),
        ]

    cfg = HalConfig()
    cfg.i2c_bus = vehicle_cfg.i2c["bus"]
    cfg.bno085_addr = vehicle_cfg.i2c["bno085_addr"]
    cfg.pca9685_addr = vehicle_cfg.i2c["pca9685_addr"]

    # RC GPIO pins
    pin_keys = sorted(vehicle_cfg.rc_gpio_pins.keys())
    for i, key in enumerate(pin_keys):
        if i < 8:
            cfg.rc_pins[i] = vehicle_cfg.rc_gpio_pins[key]
    cfg.rc_pin_count = min(len(pin_keys), 8)

    # PWM channel mapping (actuator index → PCA9685 channel)
    channel_order = [
        "motor_left", "motor_right", "tilt_left", "tilt_right",
        "aileron_left", "aileron_right", "flap_left", "flap_right", "rudder"
    ]
    for i, name in enumerate(channel_order):
        cfg.pwm_channels[i] = vehicle_cfg.pwm_channels[name]

    return cfg


def _sighup_handler(signum, frame):
    """Placeholder for PID gain hot-reload via SIGHUP."""
    log.info("SIGHUP received — PID gain reload will happen next tick")


def main():
    parser = argparse.ArgumentParser(description="VMS — Vehicle Management System")
    parser.add_argument("--config", type=Path, default=Path("config"),
                        help="Path to config directory")
    parser.add_argument("--build-dir", type=Path, default=Path("build"),
                        help="Path to C library build directory")
    parser.add_argument("--log-dir", type=Path, default=Path("logs"),
                        help="Path to telemetry log directory")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )

    # ── Load configuration ──────────────────────────────────────
    log.info("Loading configuration from %s", args.config)
    vehicle_cfg = load_vehicle_config(args.config)
    pid_cfg = load_pid_config(args.config)
    channel_cfg = load_channel_map(args.config)

    # ── Load C libraries ────────────────────────────────────────
    log.info("Loading C libraries from %s", args.build_dir)
    hal_lib, control_lib = _load_c_libraries(args.build_dir)

    # ── Initialize shared memory ────────────────────────────────
    ipc = SharedMemoryIPC(control_lib)
    ipc.init()
    log.info("Shared memory initialized")

    # Push initial PID gains
    hover_gains, cruise_gains = pid_cfg.as_gain_lists()
    ipc.update_pid_gains(hover_gains, cruise_gains)
    log.info("Initial PID gains loaded")

    # ── Initialize subsystems ───────────────────────────────────
    flight_mgr = FlightModeManager(
        transition_duration=vehicle_cfg.transition["duration_s"],
        arm_hold_time=channel_cfg.arming["hold_time_s"],
        arm_throttle_max=channel_cfg.arming["throttle_max_us"],
        arm_yaw_min=channel_cfg.arming["yaw_min_us"],
    )

    safety = SafetyMonitor(
        max_roll_deg=vehicle_cfg.safety["max_roll_deg"],
        max_pitch_deg=vehicle_cfg.safety["max_pitch_deg"],
        rc_timeout_ms=vehicle_cfg.safety["rc_timeout_ms"],
        watchdog_timeout_ms=vehicle_cfg.safety["watchdog_timeout_ms"],
    )

    telemetry = TelemetryLogger(args.log_dir, ipc)
    log.info("Telemetry logging to %s", telemetry.log_path)

    # ── Signal handlers ─────────────────────────────────────────
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGHUP, _sighup_handler)

    # ── Start control loop ──────────────────────────────────────
    hal_cfg = _build_hal_config(vehicle_cfg, control_lib)

    control_lib.vms_control_loop_start.restype = ctypes.c_int
    control_lib.vms_control_loop_start.argtypes = [ctypes.c_void_p]
    control_lib.vms_control_loop_stop.restype = None
    control_lib.vms_control_loop_stop.argtypes = []
    control_lib.vms_control_loop_is_running.restype = ctypes.c_int
    control_lib.vms_control_loop_is_running.argtypes = []

    ret = control_lib.vms_control_loop_start(ctypes.byref(hal_cfg))
    if ret != 0:
        log.error("Failed to start control loop")
        telemetry.close()
        sys.exit(1)
    log.info("Control loop started at 400Hz")

    # ── PID reload state ────────────────────────────────────────
    pid_reload_pending = False

    def _check_sighup(signum, frame):
        nonlocal pid_reload_pending
        pid_reload_pending = True

    signal.signal(signal.SIGHUP, _check_sighup)

    # ── 50Hz Python loop ────────────────────────────────────────
    log.info("Entering 50Hz orchestration loop")
    tick_count = 0

    try:
        while not g_shutdown:
            loop_start = time.monotonic()

            # Read shared state
            rc_channels = ipc.get_rc_channels()
            rc_valid = ipc.is_rc_valid()
            euler = ipc.get_euler()
            loop_counter = ipc.get_loop_counter()

            # Safety check
            is_armed = flight_mgr.mode != FlightMode.DISARMED
            safety_action = safety.check(
                loop_counter=loop_counter,
                euler_deg=euler,
                rc_valid=rc_valid,
                rc_channels=rc_channels,
                is_armed=is_armed,
            )

            if safety_action == "disarm":
                log.warning("SAFETY DISARM: %s", safety.trigger_reason)
                flight_mgr.disarm()
            elif safety_action == "failsafe":
                log.warning("SAFETY FAILSAFE: %s", safety.trigger_reason)
                flight_mgr.trigger_failsafe()

            # Flight mode update
            flight_mgr.update(rc_channels, rc_valid)

            # Write commands to shared memory
            ipc.set_flight_mode(int(flight_mgr.mode))
            ipc.set_tilt_command(flight_mgr.tilt_command)

            # Flap command from RC CH7
            if len(rc_channels) > 6:
                flap = 1.0 if rc_channels[6] > 1500 else 0.0
                ipc.set_flap_command(flap)

            # Safety override: force motors off if disarmed
            ipc.set_safety_override(flight_mgr.mode == FlightMode.DISARMED)

            # PID hot-reload via SIGHUP
            if pid_reload_pending:
                pid_reload_pending = False
                try:
                    new_pid = load_pid_config(args.config)
                    hover_gains, cruise_gains = new_pid.as_gain_lists()
                    ipc.update_pid_gains(hover_gains, cruise_gains)
                    log.info("PID gains reloaded")
                except Exception as e:
                    log.error("PID reload failed: %s", e)

            # Telemetry
            telemetry.tick(int(flight_mgr.mode), flight_mgr.tilt_command)
            if tick_count % (PYTHON_LOOP_HZ * 5) == 0:
                telemetry.flush()

            tick_count += 1

            # Sleep for remainder of period
            elapsed = time.monotonic() - loop_start
            sleep_time = PYTHON_LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception:
        log.exception("Unhandled exception in main loop")
    finally:
        log.info("Shutting down...")
        # Force motors off
        ipc.set_safety_override(True)
        ipc.set_flight_mode(int(FlightMode.DISARMED))

        control_lib.vms_control_loop_stop()
        log.info("Control loop stopped")

        telemetry.close()
        log.info("Telemetry closed. Log: %s", telemetry.log_path)


if __name__ == "__main__":
    main()
