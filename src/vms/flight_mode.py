"""
Flight mode state machine.

Manages transitions between DISARMED, VTOL_HOVER, TRANSITIONING,
FIXED_WING_CRUISE, TRANSITIONING_BACK, and FAILSAFE states.
"""

from __future__ import annotations

import time
from enum import IntEnum


class FlightMode(IntEnum):
    """Must match vms_flight_mode_t in shm_interface.h."""
    DISARMED = 0
    VTOL_HOVER = 1
    TRANSITIONING = 2
    FIXED_WING_CRUISE = 3
    TRANSITIONING_BACK = 4
    FAILSAFE = 5


class RCSwitch(IntEnum):
    """3-position switch states derived from CH5 pulse width."""
    HOVER = 0
    TRANSITION = 1
    CRUISE = 2


def classify_mode_switch(pulse_us: int, hover_max: int = 1300,
                         transition_max: int = 1700) -> RCSwitch:
    """Classify a CH5 pulse width into a switch position."""
    if pulse_us < hover_max:
        return RCSwitch.HOVER
    elif pulse_us < transition_max:
        return RCSwitch.TRANSITION
    else:
        return RCSwitch.CRUISE


class FlightModeManager:
    """
    Flight mode state machine.

    Transitions:
        DISARMED → [arm] → VTOL_HOVER
        VTOL_HOVER → [CH5 mid] → TRANSITIONING → [complete] → FIXED_WING_CRUISE
        FIXED_WING_CRUISE → [CH5 low] → TRANSITIONING_BACK → [complete] → VTOL_HOVER
        Any → FAILSAFE (on safety trigger)
        Any → DISARMED (on disarm or kill switch)
    """

    def __init__(self, transition_duration: float = 5.0,
                 arm_hold_time: float = 2.0,
                 arm_throttle_max: int = 1050,
                 arm_yaw_min: int = 1900):
        self.mode = FlightMode.DISARMED
        self.transition_duration = transition_duration
        self.arm_hold_time = arm_hold_time
        self.arm_throttle_max = arm_throttle_max
        self.arm_yaw_min = arm_yaw_min

        self._transition_start: float | None = None
        self._tilt_progress: float = 0.0  # 0=hover, 1=cruise

        self._arm_start: float | None = None

    @property
    def tilt_command(self) -> float:
        """Current tilt angle command (0.0=vertical, 1.0=horizontal)."""
        return self._tilt_progress

    def disarm(self) -> None:
        """Force disarm from any state."""
        self.mode = FlightMode.DISARMED
        self._tilt_progress = 0.0
        self._transition_start = None
        self._arm_start = None

    def trigger_failsafe(self) -> None:
        """Enter failsafe from any state."""
        self.mode = FlightMode.FAILSAFE
        self._transition_start = None

    def update(self, rc_channels: list[int], rc_valid: bool, now: float | None = None) -> FlightMode:
        """
        Run one state machine tick.

        Args:
            rc_channels: 8-element list of RC pulse widths (μs)
            rc_valid: True if RC signal is present
            now: Current time (default: time.monotonic())

        Returns:
            Current flight mode after update.
        """
        if now is None:
            now = time.monotonic()

        throttle = rc_channels[2] if len(rc_channels) > 2 else 1000
        yaw = rc_channels[3] if len(rc_channels) > 3 else 1500
        mode_sw = rc_channels[4] if len(rc_channels) > 4 else 1000
        kill_sw = rc_channels[7] if len(rc_channels) > 7 else 1000

        # Kill switch: immediate disarm from any state
        if kill_sw > 1500:
            self.disarm()
            return self.mode

        switch_pos = classify_mode_switch(mode_sw)

        if self.mode == FlightMode.DISARMED:
            self._handle_disarmed(throttle, yaw, now)

        elif self.mode == FlightMode.VTOL_HOVER:
            if switch_pos == RCSwitch.TRANSITION or switch_pos == RCSwitch.CRUISE:
                self._begin_transition_forward(now)

        elif self.mode == FlightMode.TRANSITIONING:
            self._update_transition_forward(switch_pos, now)

        elif self.mode == FlightMode.FIXED_WING_CRUISE:
            if switch_pos == RCSwitch.HOVER:
                self._begin_transition_back(now)

        elif self.mode == FlightMode.TRANSITIONING_BACK:
            self._update_transition_back(switch_pos, now)

        elif self.mode == FlightMode.FAILSAFE:
            # Stay in failsafe until explicitly disarmed
            pass

        return self.mode

    def _handle_disarmed(self, throttle: int, yaw: int, now: float) -> None:
        """Check arming sequence: throttle low + yaw right, held for arm_hold_time."""
        if throttle <= self.arm_throttle_max and yaw >= self.arm_yaw_min:
            if self._arm_start is None:
                self._arm_start = now
            elif now - self._arm_start >= self.arm_hold_time:
                self.mode = FlightMode.VTOL_HOVER
                self._arm_start = None
                self._tilt_progress = 0.0
        else:
            self._arm_start = None

    def _begin_transition_forward(self, now: float) -> None:
        self.mode = FlightMode.TRANSITIONING
        self._transition_start = now

    def _update_transition_forward(self, switch_pos: RCSwitch, now: float) -> None:
        if self._transition_start is None:
            self._transition_start = now

        elapsed = now - self._transition_start
        self._tilt_progress = min(1.0, elapsed / self.transition_duration)

        # Abort: switch back to hover
        if switch_pos == RCSwitch.HOVER:
            self._begin_transition_back(now)
            return

        # Complete
        if self._tilt_progress >= 1.0:
            self.mode = FlightMode.FIXED_WING_CRUISE
            self._transition_start = None

    def _begin_transition_back(self, now: float) -> None:
        self.mode = FlightMode.TRANSITIONING_BACK
        self._transition_start = now

    def _update_transition_back(self, switch_pos: RCSwitch, now: float) -> None:
        if self._transition_start is None:
            self._transition_start = now

        elapsed = now - self._transition_start
        self._tilt_progress = max(0.0, 1.0 - elapsed / self.transition_duration)

        # Abort: switch forward again
        if switch_pos == RCSwitch.CRUISE:
            self._begin_transition_forward(now)
            return

        # Complete
        if self._tilt_progress <= 0.0:
            self.mode = FlightMode.VTOL_HOVER
            self._transition_start = None
