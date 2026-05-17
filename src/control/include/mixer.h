#ifndef VMS_MIXER_H
#define VMS_MIXER_H

#include "shm_interface.h"

/**
 * Mixer input: normalized commands from PID + throttle + configuration.
 * All control values are -1.0 to +1.0 except throttle (0.0 to 1.0).
 */
typedef struct {
    float throttle;     /* 0.0 = off, 1.0 = full */
    float roll;         /* -1.0 = left, +1.0 = right */
    float pitch;        /* -1.0 = nose down, +1.0 = nose up */
    float yaw;          /* -1.0 = CCW, +1.0 = CW */
    float tilt;         /* 0.0 = vertical (hover), 1.0 = horizontal (cruise) */
    float flap;         /* 0.0 = retracted, 1.0 = deployed */
} vms_mixer_input_t;

/**
 * Mixer output: pulse widths in microseconds for each actuator.
 */
typedef struct {
    uint16_t channels[VMS_NUM_ACTUATORS];
} vms_mixer_output_t;

/**
 * Servo/ESC configuration for a single channel.
 */
typedef struct {
    uint16_t min_us;
    uint16_t center_us;
    uint16_t max_us;
    int      reversed;
} vms_servo_config_t;

/**
 * Full mixer configuration.
 */
typedef struct {
    vms_servo_config_t motor_left;
    vms_servo_config_t motor_right;
    vms_servo_config_t tilt_left;
    vms_servo_config_t tilt_right;
    vms_servo_config_t aileron_left;
    vms_servo_config_t aileron_right;
    vms_servo_config_t flap_left;
    vms_servo_config_t flap_right;
    vms_servo_config_t rudder;
} vms_mixer_config_t;

/**
 * Initialize mixer with servo/ESC configuration.
 */
void vms_mixer_init(const vms_mixer_config_t *config);

/**
 * Compute hover mixer outputs.
 * Roll: differential motor thrust. Pitch: differential tilt.
 * Yaw: differential motor speed. Control surfaces assist via prop wash.
 */
vms_mixer_output_t vms_mixer_hover(const vms_mixer_input_t *in);

/**
 * Compute cruise mixer outputs.
 * All attitude via aerodynamic surfaces. Motors for forward thrust only.
 */
vms_mixer_output_t vms_mixer_cruise(const vms_mixer_input_t *in);

/**
 * Compute transition mixer outputs.
 * Linear blend between hover and cruise weighted by tilt progress (in->tilt).
 */
vms_mixer_output_t vms_mixer_transition(const vms_mixer_input_t *in);

/**
 * Compute disarmed outputs: motors off, servos centered.
 */
vms_mixer_output_t vms_mixer_disarmed(void);

#endif /* VMS_MIXER_H */
