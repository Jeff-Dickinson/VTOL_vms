#include "mixer.h"

static vms_mixer_config_t g_cfg;

static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * Map a normalized value (-1..+1 or 0..+1) to a servo pulse width.
 * For centered channels (servos): -1 → min, 0 → center, +1 → max.
 * If reversed, the mapping is flipped.
 */
static uint16_t servo_map(const vms_servo_config_t *s, float norm)
{
    float v = s->reversed ? -norm : norm;
    uint16_t us;
    if (v >= 0.0f)
        us = (uint16_t)(s->center_us + v * (s->max_us - s->center_us));
    else
        us = (uint16_t)(s->center_us + v * (s->center_us - s->min_us));
    if (us < s->min_us) us = s->min_us;
    if (us > s->max_us) us = s->max_us;
    return us;
}

/**
 * Map throttle (0..1) to ESC pulse width.
 * 0 → min (off), 1 → max (full).
 */
static uint16_t motor_map(const vms_servo_config_t *s, float throttle)
{
    float t = clampf(throttle, 0.0f, 1.0f);
    uint16_t us = (uint16_t)(s->min_us + t * (s->max_us - s->min_us));
    return us;
}

void vms_mixer_init(const vms_mixer_config_t *config)
{
    g_cfg = *config;
}

vms_mixer_output_t vms_mixer_hover(const vms_mixer_input_t *in)
{
    vms_mixer_output_t out;
    float throttle = clampf(in->throttle, 0.0f, 1.0f);
    float roll  = clampf(in->roll,  -1.0f, 1.0f);
    float pitch = clampf(in->pitch, -1.0f, 1.0f);
    float yaw   = clampf(in->yaw,   -1.0f, 1.0f);

    /*
     * Hover motor mixing:
     *   Roll:  differential thrust (left+, right-)
     *   Yaw:   differential speed (CW left +, CCW right -)
     *   Base throttle applies to both.
     *
     * Motor commands: throttle ± roll_component ± yaw_component
     * Scale factor 0.3 keeps differential within usable range.
     */
    float roll_mix = roll * 0.3f;
    float yaw_mix  = yaw  * 0.3f;
    float motor_l = throttle + roll_mix + yaw_mix;
    float motor_r = throttle - roll_mix - yaw_mix;

    out.channels[ACT_MOTOR_LEFT]  = motor_map(&g_cfg.motor_left,  clampf(motor_l, 0.0f, 1.0f));
    out.channels[ACT_MOTOR_RIGHT] = motor_map(&g_cfg.motor_right, clampf(motor_r, 0.0f, 1.0f));

    /*
     * Hover tilt mixing:
     *   Pitch: differential tilt angle (tilt one forward, other back).
     *   Base tilt = 0 (vertical) in hover mode.
     */
    float pitch_tilt = pitch * 0.3f;
    out.channels[ACT_TILT_LEFT]  = servo_map(&g_cfg.tilt_left,  pitch_tilt);
    out.channels[ACT_TILT_RIGHT] = servo_map(&g_cfg.tilt_right, pitch_tilt);

    /*
     * Control surfaces assist via prop wash in hover.
     * Reduced authority (scale by 0.5).
     */
    float surf_scale = 0.5f;
    out.channels[ACT_AILERON_LEFT]  = servo_map(&g_cfg.aileron_left,  roll * surf_scale);
    out.channels[ACT_AILERON_RIGHT] = servo_map(&g_cfg.aileron_right, roll * surf_scale);
    out.channels[ACT_RUDDER]        = servo_map(&g_cfg.rudder, yaw * surf_scale);

    /* Flaps */
    out.channels[ACT_FLAP_LEFT]  = servo_map(&g_cfg.flap_left,  in->flap);
    out.channels[ACT_FLAP_RIGHT] = servo_map(&g_cfg.flap_right, in->flap);

    return out;
}

vms_mixer_output_t vms_mixer_cruise(const vms_mixer_input_t *in)
{
    vms_mixer_output_t out;
    float throttle = clampf(in->throttle, 0.0f, 1.0f);
    float roll  = clampf(in->roll,  -1.0f, 1.0f);
    float pitch = clampf(in->pitch, -1.0f, 1.0f);
    float yaw   = clampf(in->yaw,   -1.0f, 1.0f);

    /* Both motors same thrust — forward propulsion only */
    out.channels[ACT_MOTOR_LEFT]  = motor_map(&g_cfg.motor_left,  throttle);
    out.channels[ACT_MOTOR_RIGHT] = motor_map(&g_cfg.motor_right, throttle);

    /* Tilts fully horizontal in cruise */
    out.channels[ACT_TILT_LEFT]  = servo_map(&g_cfg.tilt_left,  1.0f);
    out.channels[ACT_TILT_RIGHT] = servo_map(&g_cfg.tilt_right, 1.0f);

    /*
     * Aerodynamic surfaces:
     *   Roll:  differential ailerons
     *   Pitch: symmetric aileron deflection (elevon mixing)
     *   Yaw:   rudder
     */
    float aileron_l = roll + pitch * 0.5f;
    float aileron_r = roll - pitch * 0.5f;
    out.channels[ACT_AILERON_LEFT]  = servo_map(&g_cfg.aileron_left,  clampf(aileron_l, -1.0f, 1.0f));
    out.channels[ACT_AILERON_RIGHT] = servo_map(&g_cfg.aileron_right, clampf(aileron_r, -1.0f, 1.0f));
    out.channels[ACT_RUDDER]        = servo_map(&g_cfg.rudder, yaw);

    /* Flaps */
    out.channels[ACT_FLAP_LEFT]  = servo_map(&g_cfg.flap_left,  in->flap);
    out.channels[ACT_FLAP_RIGHT] = servo_map(&g_cfg.flap_right, in->flap);

    return out;
}

vms_mixer_output_t vms_mixer_transition(const vms_mixer_input_t *in)
{
    /* Blend hover and cruise outputs based on tilt progress */
    vms_mixer_output_t hover_out  = vms_mixer_hover(in);
    vms_mixer_output_t cruise_out = vms_mixer_cruise(in);
    vms_mixer_output_t out;

    float t = clampf(in->tilt, 0.0f, 1.0f);  /* 0=full hover, 1=full cruise */

    for (int i = 0; i < VMS_NUM_ACTUATORS; i++) {
        float blended = (1.0f - t) * (float)hover_out.channels[i]
                       + t * (float)cruise_out.channels[i];
        out.channels[i] = (uint16_t)(blended + 0.5f);
    }

    /* Override tilts to match commanded tilt angle */
    out.channels[ACT_TILT_LEFT]  = servo_map(&g_cfg.tilt_left,  in->tilt);
    out.channels[ACT_TILT_RIGHT] = servo_map(&g_cfg.tilt_right, in->tilt);

    return out;
}

vms_mixer_output_t vms_mixer_disarmed(void)
{
    vms_mixer_output_t out;

    /* Motors off */
    out.channels[ACT_MOTOR_LEFT]  = g_cfg.motor_left.min_us;
    out.channels[ACT_MOTOR_RIGHT] = g_cfg.motor_right.min_us;

    /* Servos centered */
    out.channels[ACT_TILT_LEFT]    = g_cfg.tilt_left.center_us;
    out.channels[ACT_TILT_RIGHT]   = g_cfg.tilt_right.center_us;
    out.channels[ACT_AILERON_LEFT]  = g_cfg.aileron_left.center_us;
    out.channels[ACT_AILERON_RIGHT] = g_cfg.aileron_right.center_us;
    out.channels[ACT_FLAP_LEFT]    = g_cfg.flap_left.center_us;
    out.channels[ACT_FLAP_RIGHT]   = g_cfg.flap_right.center_us;
    out.channels[ACT_RUDDER]       = g_cfg.rudder.center_us;

    return out;
}
