"""Tests for the configuration loader and validator."""

from pathlib import Path

import pytest

from vms.config import (
    ConfigError,
    VehicleConfig,
    PIDConfig,
    ChannelMapConfig,
    load_vehicle_config,
    load_pid_config,
    load_channel_map,
)

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


class TestVehicleConfig:
    def test_load_valid(self):
        cfg = load_vehicle_config(CONFIG_DIR)
        assert cfg.i2c["bus"] == 1
        assert cfg.i2c["bno085_addr"] == 0x4A
        assert cfg.i2c["pca9685_addr"] == 0x40
        assert cfg.pwm_channels["motor_left"] == 0
        assert cfg.motors["pwm_min_us"] == 1000
        assert cfg.servos["tilt_left"]["center_us"] == 1500
        assert cfg.safety["max_roll_deg"] == 60.0

    def test_missing_key(self):
        with pytest.raises(ConfigError, match="Missing key"):
            VehicleConfig({})

    def test_missing_servo(self):
        data = {
            "i2c": {"bus": 1, "bno085_addr": 0x4A, "pca9685_addr": 0x40},
            "pwm_channels": {
                "motor_left": 0, "motor_right": 1, "tilt_left": 2,
                "tilt_right": 3, "aileron_left": 4, "aileron_right": 5,
                "flap_left": 6, "flap_right": 7, "rudder": 8,
            },
            "motors": {"pwm_min_us": 1000, "pwm_max_us": 2000, "pwm_arm_us": 1000},
            "servos": {},  # Missing servos
            "rc_gpio_pins": {},
            "safety": {"max_roll_deg": 60, "max_pitch_deg": 60,
                        "rc_timeout_ms": 500, "watchdog_timeout_ms": 50},
            "transition": {"duration_s": 5.0},
        }
        with pytest.raises(ConfigError, match="Missing servo"):
            VehicleConfig(data)

    def test_invalid_servo_range(self):
        servo = {"min_us": 2000, "center_us": 1500, "max_us": 1000, "reversed": False}
        data = {
            "i2c": {"bus": 1, "bno085_addr": 0x4A, "pca9685_addr": 0x40},
            "pwm_channels": {
                "motor_left": 0, "motor_right": 1, "tilt_left": 2,
                "tilt_right": 3, "aileron_left": 4, "aileron_right": 5,
                "flap_left": 6, "flap_right": 7, "rudder": 8,
            },
            "motors": {"pwm_min_us": 1000, "pwm_max_us": 2000, "pwm_arm_us": 1000},
            "servos": {
                "tilt_left": servo, "tilt_right": servo,
                "aileron_left": servo, "aileron_right": servo,
                "flap_left": servo, "flap_right": servo,
                "rudder": servo,
            },
            "rc_gpio_pins": {},
            "safety": {"max_roll_deg": 60, "max_pitch_deg": 60,
                        "rc_timeout_ms": 500, "watchdog_timeout_ms": 50},
            "transition": {"duration_s": 5.0},
        }
        with pytest.raises(ConfigError, match="min_us must be < max_us"):
            VehicleConfig(data)


class TestPIDConfig:
    def test_load_valid(self):
        cfg = load_pid_config(CONFIG_DIR)
        assert cfg.hover["roll"]["kp"] == 4.0
        assert cfg.cruise["yaw"]["ki"] == 0.1

    def test_as_gain_lists(self):
        cfg = load_pid_config(CONFIG_DIR)
        hover, cruise = cfg.as_gain_lists()
        assert len(hover) == 3
        assert len(cruise) == 3
        assert hover[0]["kp"] == 4.0  # roll
        assert hover[2]["kp"] == 2.0  # yaw

    def test_missing_axis(self):
        with pytest.raises(ConfigError):
            PIDConfig({"hover": {}, "cruise": {}})

    def test_negative_gain(self):
        axis = {"kp": -1.0, "ki": 0, "kd": 0,
                "output_min": -1, "output_max": 1, "integral_max": 0.5}
        with pytest.raises(ConfigError, match="non-negative"):
            PIDConfig({
                "hover": {"roll": axis, "pitch": axis, "yaw": axis},
                "cruise": {"roll": axis, "pitch": axis, "yaw": axis},
            })


class TestChannelMapConfig:
    def test_load_valid(self):
        cfg = load_channel_map(CONFIG_DIR)
        assert cfg.channels["roll"]["channel"] == 1
        assert cfg.channels["throttle"]["min_us"] == 1000
        assert cfg.arming["hold_time_s"] == 2.0

    def test_missing_channel(self):
        with pytest.raises(ConfigError):
            ChannelMapConfig({"channels": {}, "arming": {}})


class TestFileLoading:
    def test_missing_file(self):
        with pytest.raises(ConfigError, match="not found"):
            load_vehicle_config(Path("/nonexistent"))
