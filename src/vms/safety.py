"""
Safety monitor.

Checks for conditions that require failsafe or disarm:
- Control loop watchdog (loop counter must advance)
- RC signal loss
- Attitude limits exceeded
- Kill switch
"""

from __future__ import annotations

import time


class SafetyMonitor:
    """
    Monitors safety conditions at 50Hz from the Python orchestration loop.

    When a safety condition is triggered, returns the appropriate action
    for the flight mode manager to execute.
    """

    def __init__(
        self,
        max_roll_deg: float = 60.0,
        max_pitch_deg: float = 60.0,
        rc_timeout_ms: int = 500,
        watchdog_timeout_ms: int = 50,
    ):
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg
        self.rc_timeout_ms = rc_timeout_ms
        self.watchdog_timeout_ms = watchdog_timeout_ms

        self._last_loop_counter: int = 0
        self._last_loop_check_time: float = 0.0
        self._triggered: bool = False
        self._trigger_reason: str = ""

    @property
    def is_triggered(self) -> bool:
        return self._triggered

    @property
    def trigger_reason(self) -> str:
        return self._trigger_reason

    def reset(self) -> None:
        """Reset safety state (e.g., after disarm)."""
        self._triggered = False
        self._trigger_reason = ""
        self._last_loop_counter = 0
        self._last_loop_check_time = 0.0

    def check(
        self,
        loop_counter: int,
        euler_deg: tuple[float, float, float],
        rc_valid: bool,
        rc_channels: list[int],
        is_armed: bool,
        now: float | None = None,
    ) -> str | None:
        """
        Run all safety checks.

        Returns:
            None if all checks pass.
            "failsafe" if a recoverable safety condition is triggered.
            "disarm" if an immediate disarm is required.
        """
        if now is None:
            now = time.monotonic()

        if not is_armed:
            self.reset()
            return None

        # Kill switch (CH8 > 1500)
        if len(rc_channels) > 7 and rc_channels[7] > 1500:
            self._triggered = True
            self._trigger_reason = "kill_switch"
            return "disarm"

        # RC signal loss
        if not rc_valid:
            self._triggered = True
            self._trigger_reason = "rc_signal_lost"
            return "failsafe"

        # Control loop watchdog
        if self._last_loop_check_time > 0.0:
            elapsed_ms = (now - self._last_loop_check_time) * 1000.0
            if elapsed_ms > self.watchdog_timeout_ms and loop_counter == self._last_loop_counter:
                self._triggered = True
                self._trigger_reason = "control_loop_stalled"
                return "failsafe"
        self._last_loop_counter = loop_counter
        self._last_loop_check_time = now

        # Attitude limits
        roll, pitch, _ = euler_deg
        if abs(roll) > self.max_roll_deg:
            self._triggered = True
            self._trigger_reason = f"roll_exceeded ({roll:.1f} deg)"
            return "failsafe"
        if abs(pitch) > self.max_pitch_deg:
            self._triggered = True
            self._trigger_reason = f"pitch_exceeded ({pitch:.1f} deg)"
            return "failsafe"

        return None
