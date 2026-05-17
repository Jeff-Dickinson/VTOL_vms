#ifndef VMS_PID_H
#define VMS_PID_H

#include "shm_interface.h"

typedef struct {
    /* Configuration */
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;     /* Anti-windup: clamp |integral| to this value */

    /* State */
    float integral;
    float prev_measurement; /* For derivative-on-measurement */
    int   initialized;      /* 0 until first update */
} vms_pid_t;

/**
 * Initialize a PID controller with the given gains.
 * Resets all internal state.
 */
void vms_pid_init(vms_pid_t *pid, const vms_pid_gains_t *gains);

/**
 * Load new gains into a PID controller without resetting state.
 * Use for hot-reload of tuning parameters.
 */
void vms_pid_set_gains(vms_pid_t *pid, const vms_pid_gains_t *gains);

/**
 * Compute one PID iteration.
 *
 * Uses derivative-on-measurement (not derivative-on-error) to avoid
 * derivative kick on setpoint changes.
 *
 * @param pid         Controller instance
 * @param setpoint    Desired value
 * @param measurement Current measured value
 * @param dt          Time step in seconds (must be > 0)
 * @return            Control output, clamped to [output_min, output_max]
 */
float vms_pid_update(vms_pid_t *pid, float setpoint, float measurement, float dt);

/**
 * Reset PID internal state (integral, derivative memory).
 * Call when disarming or switching modes to avoid integral windup carryover.
 */
void vms_pid_reset(vms_pid_t *pid);

#endif /* VMS_PID_H */
