# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

VTOL_POC — Vehicle Management System (VMS) for a tilt-rotor VTOL drone proof of concept. Runs on Raspberry Pi 4B with BNO085 IMU, PCA9685 PWM driver, FlySky FS-R9B receiver, 2 motors, 2 tilt servos, and 5 control surface servos.

## Architecture

- **C layer** (400Hz real-time): PID control, mixer, sensor reading, PWM output. Runs as a SCHED_FIFO thread pinned to CPU core 3.
- **Python layer** (50Hz orchestration): Flight mode state machine, safety monitoring, telemetry logging, configuration.
- **IPC**: Single shared memory struct (`vms_shared_state_t` in `src/bindings/shm_interface.h`) with C11 atomics. C writes sensor/control data; Python writes flight mode commands.

## Build Commands

```bash
# C libraries (builds libvms_hal.so + libvms_control.so)
mkdir -p build && cd build && cmake .. && make

# Python package (editable install)
pip install -e .

# Run C unit tests
cd build && ctest

# Run Python tests
PYTHONPATH=src pytest tests/test_config.py tests/test_flight_mode.py

# Compile and run PID tests standalone (no cmake needed)
cc -std=c11 -O2 -I src/control/include -I src/bindings -o /tmp/test_pid tests/test_pid.c src/control/src/pid.c -lm && /tmp/test_pid

# Compile and run mixer tests standalone
cc -std=c11 -O2 -I src/control/include -I src/hal/include -I src/bindings -o /tmp/test_mixer tests/test_mixer.c src/control/src/mixer.c -lm && /tmp/test_mixer
```

## Run

```bash
# On Pi (requires sudo for GPIO/I2C access)
sudo vms-start --config config/

# Or directly
sudo python -m vms.main --config config/ --build-dir build/

# Deploy to Pi from dev machine
./scripts/deploy.sh pi@vtol.local
```

## Key Files

- `src/bindings/shm_interface.h` — Shared memory struct (the C/Python contract). Changes here require updating `src/vms/ipc.py` to match.
- `src/control/src/pid.c` — PID controller with anti-windup and derivative-on-measurement.
- `src/control/src/mixer.c` — Hover/cruise/transition mixers (9 actuator outputs).
- `src/control/src/control_loop.c` — 400Hz RT loop: IMU read -> PID -> mixer -> PWM write.
- `src/vms/flight_mode.py` — State machine: DISARMED -> VTOL_HOVER -> TRANSITIONING -> FIXED_WING_CRUISE.
- `src/vms/main.py` — Entry point: boots system, spawns C thread, runs 50Hz Python loop.
- `config/pid_gains.yaml` — PID tuning parameters (hot-reloadable via SIGHUP).

## Conventions

- C code: C11 standard, `vms_` prefix for all public symbols.
- Python: 3.9+ compatible (`from __future__ import annotations` for type hints).
- Config: YAML files in `config/`. Validated on load by `src/vms/config.py`.
- Safety: Kill switch (RC CH8) disarms from any state. Motor test tools require explicit confirmation.
