"""
Shared memory Python-side wrapper (ctypes).

Maps the vms_shared_state_t struct from shm_interface.h so Python can
read sensor/control data and write flight mode commands to the C thread.
"""

import ctypes

VMS_RC_CHANNELS = 8
VMS_NUM_ACTUATORS = 9
VMS_PID_AXES = 3

# Flight mode enum values (must match shm_interface.h)
MODE_DISARMED = 0
MODE_VTOL_HOVER = 1
MODE_TRANSITIONING = 2
MODE_FIXED_WING_CRUISE = 3
MODE_TRANSITIONING_BACK = 4
MODE_FAILSAFE = 5

# Actuator indices (must match shm_interface.h)
ACT_MOTOR_LEFT = 0
ACT_MOTOR_RIGHT = 1
ACT_TILT_LEFT = 2
ACT_TILT_RIGHT = 3
ACT_AILERON_LEFT = 4
ACT_AILERON_RIGHT = 5
ACT_FLAP_LEFT = 6
ACT_FLAP_RIGHT = 7
ACT_RUDDER = 8


class PIDGains(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("output_min", ctypes.c_float),
        ("output_max", ctypes.c_float),
        ("integral_max", ctypes.c_float),
    ]


class SharedState(ctypes.Structure):
    """
    Must exactly match the memory layout of vms_shared_state_t in shm_interface.h.
    """

    _fields_ = [
        # C → Python
        ("loop_counter", ctypes.c_uint64),
        ("loop_dt_us", ctypes.c_uint32),
        ("quaternion", ctypes.c_float * 4),
        ("euler_deg", ctypes.c_float * 3),
        ("gyro_dps", ctypes.c_float * 3),
        ("rc_channels", ctypes.c_uint16 * VMS_RC_CHANNELS),
        ("rc_valid", ctypes.c_uint8),
        ("pid_output", ctypes.c_float * VMS_PID_AXES),
        ("actuator_us", ctypes.c_uint16 * VMS_NUM_ACTUATORS),
        # Python → C
        ("flight_mode", ctypes.c_int32),
        ("tilt_command", ctypes.c_float),
        ("flap_command", ctypes.c_float),
        ("safety_override", ctypes.c_uint8),
        ("pid_gains_hover", PIDGains * VMS_PID_AXES),
        ("pid_gains_cruise", PIDGains * VMS_PID_AXES),
        ("pid_gains_updated", ctypes.c_uint8),
    ]


class SharedMemoryIPC:
    """Interface to the shared state managed by the C control library."""

    def __init__(self, control_lib: ctypes.CDLL):
        self._lib = control_lib
        self._lib.vms_shm_get.restype = ctypes.POINTER(SharedState)
        self._lib.vms_shm_get.argtypes = []
        self._lib.vms_shm_init.restype = None
        self._lib.vms_shm_init.argtypes = []

    def init(self) -> None:
        """Initialize shared memory (zero everything, set defaults)."""
        self._lib.vms_shm_init()

    @property
    def state(self) -> SharedState:
        """Get a reference to the shared state struct."""
        return self._lib.vms_shm_get().contents

    # ── Convenience readers (C → Python) ─────────────────────────

    def get_euler(self) -> tuple[float, float, float]:
        s = self.state
        return (s.euler_deg[0], s.euler_deg[1], s.euler_deg[2])

    def get_quaternion(self) -> tuple[float, float, float, float]:
        s = self.state
        return (s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3])

    def get_gyro(self) -> tuple[float, float, float]:
        s = self.state
        return (s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2])

    def get_rc_channels(self) -> list[int]:
        s = self.state
        return [s.rc_channels[i] for i in range(VMS_RC_CHANNELS)]

    def is_rc_valid(self) -> bool:
        return bool(self.state.rc_valid)

    def get_loop_counter(self) -> int:
        return self.state.loop_counter

    def get_loop_dt_us(self) -> int:
        return self.state.loop_dt_us

    def get_pid_outputs(self) -> tuple[float, float, float]:
        s = self.state
        return (s.pid_output[0], s.pid_output[1], s.pid_output[2])

    def get_actuator_us(self) -> list[int]:
        s = self.state
        return [s.actuator_us[i] for i in range(VMS_NUM_ACTUATORS)]

    # ── Convenience writers (Python → C) ─────────────────────────

    def set_flight_mode(self, mode: int) -> None:
        self.state.flight_mode = mode

    def set_tilt_command(self, tilt: float) -> None:
        self.state.tilt_command = max(0.0, min(1.0, tilt))

    def set_flap_command(self, flap: float) -> None:
        self.state.flap_command = max(0.0, min(1.0, flap))

    def set_safety_override(self, override: bool) -> None:
        self.state.safety_override = 1 if override else 0

    def update_pid_gains(
        self,
        hover_gains: list[dict],
        cruise_gains: list[dict],
    ) -> None:
        """
        Push new PID gains to the C thread.

        Each list should have 3 dicts (roll, pitch, yaw) with keys:
        kp, ki, kd, output_min, output_max, integral_max.
        """
        s = self.state
        for i, g in enumerate(hover_gains):
            s.pid_gains_hover[i].kp = g["kp"]
            s.pid_gains_hover[i].ki = g["ki"]
            s.pid_gains_hover[i].kd = g["kd"]
            s.pid_gains_hover[i].output_min = g["output_min"]
            s.pid_gains_hover[i].output_max = g["output_max"]
            s.pid_gains_hover[i].integral_max = g["integral_max"]
        for i, g in enumerate(cruise_gains):
            s.pid_gains_cruise[i].kp = g["kp"]
            s.pid_gains_cruise[i].ki = g["ki"]
            s.pid_gains_cruise[i].kd = g["kd"]
            s.pid_gains_cruise[i].output_min = g["output_min"]
            s.pid_gains_cruise[i].output_max = g["output_max"]
            s.pid_gains_cruise[i].integral_max = g["integral_max"]
        # Signal C thread to pick up new gains
        s.pid_gains_updated = 1
