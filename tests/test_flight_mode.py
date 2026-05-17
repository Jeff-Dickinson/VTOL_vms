"""Tests for the flight mode state machine."""

import pytest

from vms.flight_mode import FlightMode, FlightModeManager, classify_mode_switch, RCSwitch


def make_rc(throttle=1000, yaw=1500, mode_sw=1000, kill=1000):
    """Helper to build RC channel array."""
    return [1500, 1500, throttle, yaw, mode_sw, 1500, 1000, kill]


class TestClassifyModeSwitch:
    def test_hover(self):
        assert classify_mode_switch(1000) == RCSwitch.HOVER

    def test_transition(self):
        assert classify_mode_switch(1500) == RCSwitch.TRANSITION

    def test_cruise(self):
        assert classify_mode_switch(2000) == RCSwitch.CRUISE

    def test_boundaries(self):
        assert classify_mode_switch(1299) == RCSwitch.HOVER
        assert classify_mode_switch(1300) == RCSwitch.TRANSITION
        assert classify_mode_switch(1699) == RCSwitch.TRANSITION
        assert classify_mode_switch(1700) == RCSwitch.CRUISE


class TestArming:
    def test_starts_disarmed(self):
        fm = FlightModeManager()
        assert fm.mode == FlightMode.DISARMED

    def test_arm_sequence(self):
        fm = FlightModeManager(arm_hold_time=2.0)
        rc = make_rc(throttle=1000, yaw=2000)

        # Not armed yet at t=0
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.DISARMED

        # Not armed at t=1.9
        fm.update(rc, True, now=1.9)
        assert fm.mode == FlightMode.DISARMED

        # Armed at t=2.0
        fm.update(rc, True, now=2.0)
        assert fm.mode == FlightMode.VTOL_HOVER

    def test_arm_interrupted(self):
        fm = FlightModeManager(arm_hold_time=2.0)
        rc_arm = make_rc(throttle=1000, yaw=2000)
        rc_idle = make_rc(throttle=1000, yaw=1500)

        fm.update(rc_arm, True, now=0.0)
        fm.update(rc_idle, True, now=1.0)  # Release yaw
        fm.update(rc_arm, True, now=1.5)   # Resume
        fm.update(rc_arm, True, now=3.0)   # 1.5s since resume, not 2s
        assert fm.mode == FlightMode.DISARMED

        fm.update(rc_arm, True, now=3.5)   # 2.0s since resume
        assert fm.mode == FlightMode.VTOL_HOVER

    def test_throttle_too_high(self):
        fm = FlightModeManager(arm_hold_time=2.0, arm_throttle_max=1050)
        rc = make_rc(throttle=1200, yaw=2000)

        fm.update(rc, True, now=0.0)
        fm.update(rc, True, now=5.0)
        assert fm.mode == FlightMode.DISARMED


class TestKillSwitch:
    def test_kill_from_hover(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.VTOL_HOVER

        rc = make_rc(kill=2000)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.DISARMED

    def test_kill_from_cruise(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.FIXED_WING_CRUISE

        rc = make_rc(kill=2000)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.DISARMED

    def test_kill_from_transition(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.TRANSITIONING

        rc = make_rc(kill=1600)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.DISARMED


class TestTransition:
    def test_hover_to_cruise(self):
        fm = FlightModeManager(transition_duration=5.0)
        fm.mode = FlightMode.VTOL_HOVER

        # Switch to transition
        rc = make_rc(mode_sw=1500)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.TRANSITIONING
        assert fm.tilt_command == 0.0

        # Midway
        fm.update(rc, True, now=2.5)
        assert fm.mode == FlightMode.TRANSITIONING
        assert abs(fm.tilt_command - 0.5) < 0.01

        # Complete
        fm.update(rc, True, now=5.0)
        assert fm.mode == FlightMode.FIXED_WING_CRUISE
        assert fm.tilt_command >= 1.0

    def test_cruise_to_hover(self):
        fm = FlightModeManager(transition_duration=5.0)
        fm.mode = FlightMode.FIXED_WING_CRUISE
        fm._tilt_progress = 1.0

        # Switch back
        rc = make_rc(mode_sw=1000)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.TRANSITIONING_BACK

        fm.update(rc, True, now=2.5)
        assert abs(fm.tilt_command - 0.5) < 0.01

        fm.update(rc, True, now=5.0)
        assert fm.mode == FlightMode.VTOL_HOVER
        assert fm.tilt_command <= 0.0

    def test_transition_abort(self):
        fm = FlightModeManager(transition_duration=5.0)
        fm.mode = FlightMode.VTOL_HOVER

        # Start forward transition
        rc_trans = make_rc(mode_sw=1500)
        fm.update(rc_trans, True, now=0.0)
        fm.update(rc_trans, True, now=2.0)
        assert fm.mode == FlightMode.TRANSITIONING

        # Abort — switch back to hover
        rc_hover = make_rc(mode_sw=1000)
        fm.update(rc_hover, True, now=2.0)
        assert fm.mode == FlightMode.TRANSITIONING_BACK


class TestFailsafe:
    def test_failsafe_from_hover(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.VTOL_HOVER
        fm.trigger_failsafe()
        assert fm.mode == FlightMode.FAILSAFE

    def test_failsafe_stays(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.FAILSAFE
        rc = make_rc(mode_sw=1500)
        fm.update(rc, True, now=0.0)
        assert fm.mode == FlightMode.FAILSAFE

    def test_disarm_from_failsafe(self):
        fm = FlightModeManager()
        fm.mode = FlightMode.FAILSAFE
        fm.disarm()
        assert fm.mode == FlightMode.DISARMED
