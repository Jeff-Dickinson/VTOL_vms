#ifndef VMS_SHM_INTERFACE_H
#define VMS_SHM_INTERFACE_H

#include <stdint.h>
#include <stdatomic.h>

#define VMS_RC_CHANNELS     8
#define VMS_NUM_ACTUATORS   9   /* 2 motors + 2 tilt + 4 surfaces + 1 rudder */
#define VMS_PID_AXES        3   /* roll, pitch, yaw */

/* Flight mode enum — shared between C and Python */
typedef enum {
    MODE_DISARMED           = 0,
    MODE_VTOL_HOVER         = 1,
    MODE_TRANSITIONING      = 2,
    MODE_FIXED_WING_CRUISE  = 3,
    MODE_TRANSITIONING_BACK = 4,
    MODE_FAILSAFE           = 5,
} vms_flight_mode_t;

/* Actuator index mapping */
typedef enum {
    ACT_MOTOR_LEFT      = 0,
    ACT_MOTOR_RIGHT     = 1,
    ACT_TILT_LEFT       = 2,
    ACT_TILT_RIGHT      = 3,
    ACT_AILERON_LEFT    = 4,
    ACT_AILERON_RIGHT   = 5,
    ACT_FLAP_LEFT       = 6,
    ACT_FLAP_RIGHT      = 7,
    ACT_RUDDER          = 8,
} vms_actuator_id_t;

/* PID gains for a single axis */
typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;
} vms_pid_gains_t;

/*
 * Shared state struct — single instance mapped by both C and Python.
 *
 * Layout rules:
 *   - C writes sensor/control data at 400Hz, Python reads at 50Hz.
 *   - Python writes commands/config, C reads each loop iteration.
 *   - _Atomic used on cross-thread single values for visibility guarantees.
 *   - Aggregate arrays (quaternion, rc_channels, etc.) are written atomically
 *     by one thread only — minor torn reads acceptable for telemetry.
 */
typedef struct {
    /* ── C → Python (control thread writes, Python reads) ────────── */

    /* Loop timing */
    _Atomic uint64_t loop_counter;          /* Monotonic tick count */
    _Atomic uint32_t loop_dt_us;            /* Actual loop period (μs) */

    /* IMU data (BNO085 quaternion output) */
    float quaternion[4];                    /* [w, x, y, z] */
    float euler_deg[3];                     /* [roll, pitch, yaw] degrees */
    float gyro_dps[3];                      /* [roll, pitch, yaw] deg/s */

    /* RC input (raw pulse widths in μs) */
    uint16_t rc_channels[VMS_RC_CHANNELS];
    _Atomic uint8_t rc_valid;               /* 1 = receiving signal */

    /* PID outputs (normalized -1..+1) */
    float pid_output[VMS_PID_AXES];         /* [roll, pitch, yaw] */

    /* Final actuator outputs (μs pulse widths to PCA9685) */
    uint16_t actuator_us[VMS_NUM_ACTUATORS];

    /* ── Python → C (Python writes, control thread reads) ────────── */

    /* Flight mode command */
    _Atomic int32_t flight_mode;            /* vms_flight_mode_t */

    /* Tilt angle command: 0.0 = vertical (hover), 1.0 = horizontal (cruise) */
    _Atomic float tilt_command;

    /* Flap command: 0.0 = retracted, 1.0 = deployed */
    _Atomic float flap_command;

    /* Safety override: 1 = force motors off */
    _Atomic uint8_t safety_override;

    /* PID gain hot-reload */
    vms_pid_gains_t pid_gains_hover[VMS_PID_AXES];
    vms_pid_gains_t pid_gains_cruise[VMS_PID_AXES];
    _Atomic uint8_t pid_gains_updated;      /* Set to 1 by Python, cleared by C */

} vms_shared_state_t;

/**
 * Get pointer to the shared state singleton.
 */
vms_shared_state_t *vms_shm_get(void);

/**
 * Initialize shared state to defaults (zeroed, DISARMED mode).
 */
void vms_shm_init(void);

#endif /* VMS_SHM_INTERFACE_H */
