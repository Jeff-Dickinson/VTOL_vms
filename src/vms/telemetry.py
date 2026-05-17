"""
Telemetry logger.

Logs flight data from shared memory at 50Hz to a CSV file.
Each row contains a timestamp and all key state variables.
"""

from __future__ import annotations

import csv
import time
from pathlib import Path

from vms.ipc import SharedMemoryIPC, VMS_RC_CHANNELS, VMS_NUM_ACTUATORS, VMS_PID_AXES


class TelemetryLogger:
    """Writes flight data to a CSV file at each tick."""

    HEADER = [
        "timestamp",
        "loop_counter",
        "loop_dt_us",
        "flight_mode",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "gyro_roll_dps",
        "gyro_pitch_dps",
        "gyro_yaw_dps",
        "qw", "qx", "qy", "qz",
        "rc_ch1", "rc_ch2", "rc_ch3", "rc_ch4",
        "rc_ch5", "rc_ch6", "rc_ch7", "rc_ch8",
        "rc_valid",
        "pid_roll", "pid_pitch", "pid_yaw",
        "tilt_command",
        "act_motor_l", "act_motor_r",
        "act_tilt_l", "act_tilt_r",
        "act_ail_l", "act_ail_r",
        "act_flap_l", "act_flap_r",
        "act_rudder",
    ]

    def __init__(self, log_dir: Path, ipc: SharedMemoryIPC):
        self._ipc = ipc
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._path = log_dir / f"flight_{stamp}.csv"
        self._file = open(self._path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._start_time = time.monotonic()

    @property
    def log_path(self) -> Path:
        return self._path

    def tick(self, flight_mode: int, tilt_command: float) -> None:
        """Log one row of telemetry data."""
        s = self._ipc.state
        t = time.monotonic() - self._start_time

        euler = (s.euler_deg[0], s.euler_deg[1], s.euler_deg[2])
        gyro = (s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2])
        quat = (s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3])
        rc = [s.rc_channels[i] for i in range(VMS_RC_CHANNELS)]
        pid = [s.pid_output[i] for i in range(VMS_PID_AXES)]
        act = [s.actuator_us[i] for i in range(VMS_NUM_ACTUATORS)]

        row = [
            f"{t:.4f}",
            s.loop_counter,
            s.loop_dt_us,
            flight_mode,
            f"{euler[0]:.2f}", f"{euler[1]:.2f}", f"{euler[2]:.2f}",
            f"{gyro[0]:.2f}", f"{gyro[1]:.2f}", f"{gyro[2]:.2f}",
            f"{quat[0]:.4f}", f"{quat[1]:.4f}", f"{quat[2]:.4f}", f"{quat[3]:.4f}",
            *rc,
            s.rc_valid,
            f"{pid[0]:.4f}", f"{pid[1]:.4f}", f"{pid[2]:.4f}",
            f"{tilt_command:.3f}",
            *act,
        ]
        self._writer.writerow(row)

    def flush(self) -> None:
        """Flush buffered data to disk."""
        self._file.flush()

    def close(self) -> None:
        """Close the log file."""
        self._file.close()
