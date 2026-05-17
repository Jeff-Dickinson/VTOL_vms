"""
Configuration loader and validator.

Loads vehicle.yaml, pid_gains.yaml, and channel_map.yaml.
Supports hot-reload of PID gains via SIGHUP.
"""

from pathlib import Path
from typing import Any

import yaml


class ConfigError(Exception):
    """Raised when configuration validation fails."""


def _require_keys(d: dict, keys: list[str], context: str) -> None:
    for k in keys:
        if k not in d:
            raise ConfigError(f"Missing key '{k}' in {context}")


def _require_type(val: Any, typ: type, name: str) -> None:
    if not isinstance(val, typ):
        raise ConfigError(f"'{name}' must be {typ.__name__}, got {type(val).__name__}")


def _validate_servo(servo: dict, name: str) -> None:
    _require_keys(servo, ["min_us", "center_us", "max_us", "reversed"], f"servo '{name}'")
    for k in ("min_us", "center_us", "max_us"):
        v = servo[k]
        if not isinstance(v, (int, float)) or v < 500 or v > 2500:
            raise ConfigError(f"servo '{name}.{k}' must be 500-2500, got {v}")
    if servo["min_us"] >= servo["max_us"]:
        raise ConfigError(f"servo '{name}': min_us must be < max_us")


def _validate_pid_axis(axis: dict, name: str) -> None:
    _require_keys(axis, ["kp", "ki", "kd", "output_min", "output_max", "integral_max"], name)
    for k in ("kp", "ki", "kd"):
        v = axis[k]
        if not isinstance(v, (int, float)) or v < 0:
            raise ConfigError(f"'{name}.{k}' must be non-negative, got {v}")
    if axis["output_min"] >= axis["output_max"]:
        raise ConfigError(f"'{name}': output_min must be < output_max")


class VehicleConfig:
    """Validated vehicle configuration from vehicle.yaml."""

    def __init__(self, data: dict):
        _require_keys(data, ["i2c", "pwm_channels", "motors", "servos",
                             "rc_gpio_pins", "safety", "transition"], "vehicle.yaml")

        self.i2c = data["i2c"]
        _require_keys(self.i2c, ["bus", "bno085_addr", "pca9685_addr"], "i2c")

        self.pwm_channels = data["pwm_channels"]
        expected_channels = [
            "motor_left", "motor_right", "tilt_left", "tilt_right",
            "aileron_left", "aileron_right", "flap_left", "flap_right", "rudder"
        ]
        _require_keys(self.pwm_channels, expected_channels, "pwm_channels")

        self.motors = data["motors"]
        _require_keys(self.motors, ["pwm_min_us", "pwm_max_us", "pwm_arm_us"], "motors")

        self.servos = data["servos"]
        for name in ["tilt_left", "tilt_right", "aileron_left", "aileron_right",
                      "flap_left", "flap_right", "rudder"]:
            if name not in self.servos:
                raise ConfigError(f"Missing servo config '{name}'")
            _validate_servo(self.servos[name], name)

        self.rc_gpio_pins = data["rc_gpio_pins"]
        self.safety = data["safety"]
        _require_keys(self.safety, ["max_roll_deg", "max_pitch_deg",
                                     "rc_timeout_ms", "watchdog_timeout_ms"], "safety")

        self.transition = data["transition"]
        _require_keys(self.transition, ["duration_s"], "transition")


class PIDConfig:
    """Validated PID gains from pid_gains.yaml."""

    def __init__(self, data: dict):
        _require_keys(data, ["hover", "cruise"], "pid_gains.yaml")

        self.hover = {}
        self.cruise = {}
        for axis in ("roll", "pitch", "yaw"):
            if axis not in data["hover"]:
                raise ConfigError(f"Missing hover PID axis '{axis}'")
            _validate_pid_axis(data["hover"][axis], f"hover.{axis}")
            self.hover[axis] = data["hover"][axis]

            if axis not in data["cruise"]:
                raise ConfigError(f"Missing cruise PID axis '{axis}'")
            _validate_pid_axis(data["cruise"][axis], f"cruise.{axis}")
            self.cruise[axis] = data["cruise"][axis]

    def as_gain_lists(self) -> tuple[list[dict], list[dict]]:
        """Return gains in [roll, pitch, yaw] order for IPC."""
        axes = ("roll", "pitch", "yaw")
        return (
            [self.hover[a] for a in axes],
            [self.cruise[a] for a in axes],
        )


class ChannelMapConfig:
    """Validated RC channel map from channel_map.yaml."""

    def __init__(self, data: dict):
        _require_keys(data, ["channels", "arming"], "channel_map.yaml")

        ch = data["channels"]
        _require_keys(ch, ["roll", "pitch", "throttle", "yaw",
                           "flight_mode", "tilt_override", "flaps", "kill_switch"],
                      "channels")

        self.channels = ch
        self.arming = data["arming"]
        _require_keys(self.arming, ["throttle_max_us", "yaw_min_us", "hold_time_s"],
                      "arming")


def load_yaml(path: Path) -> dict:
    """Load a YAML file and return parsed dict."""
    if not path.exists():
        raise ConfigError(f"Config file not found: {path}")
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ConfigError(f"Config file must contain a YAML mapping: {path}")
    return data


def load_vehicle_config(config_dir: Path) -> VehicleConfig:
    return VehicleConfig(load_yaml(config_dir / "vehicle.yaml"))


def load_pid_config(config_dir: Path) -> PIDConfig:
    return PIDConfig(load_yaml(config_dir / "pid_gains.yaml"))


def load_channel_map(config_dir: Path) -> ChannelMapConfig:
    return ChannelMapConfig(load_yaml(config_dir / "channel_map.yaml"))
