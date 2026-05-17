#include "pid.h"

static float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void vms_pid_init(vms_pid_t *pid, const vms_pid_gains_t *gains)
{
    pid->kp = gains->kp;
    pid->ki = gains->ki;
    pid->kd = gains->kd;
    pid->output_min = gains->output_min;
    pid->output_max = gains->output_max;
    pid->integral_max = gains->integral_max;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
}

void vms_pid_set_gains(vms_pid_t *pid, const vms_pid_gains_t *gains)
{
    pid->kp = gains->kp;
    pid->ki = gains->ki;
    pid->kd = gains->kd;
    pid->output_min = gains->output_min;
    pid->output_max = gains->output_max;
    pid->integral_max = gains->integral_max;
    /* Re-clamp integral with new limit */
    pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
}

float vms_pid_update(vms_pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup clamping */
    pid->integral += error * dt;
    pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    /* Derivative on measurement (not error) to avoid setpoint kick */
    float d_term = 0.0f;
    if (pid->initialized) {
        float d_measurement = (measurement - pid->prev_measurement) / dt;
        d_term = -pid->kd * d_measurement;   /* Negative: oppose rate of change */
    }
    pid->prev_measurement = measurement;
    pid->initialized = 1;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    return clampf(output, pid->output_min, pid->output_max);
}

void vms_pid_reset(vms_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
}
