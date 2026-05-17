#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "mixer.h"

#define ASSERT_EQ(a, b) do { \
    uint16_t _a = (a), _b = (b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %u != %u\n", __FILE__, __LINE__, _a, _b); \
        assert(0); \
    } \
} while(0)

#define ASSERT_NEAR_U16(a, b, tol) do { \
    uint16_t _a = (a), _b = (b); \
    if (abs((int)_a - (int)_b) > (int)(tol)) { \
        fprintf(stderr, "FAIL %s:%d: %u != %u (tol %u)\n", \
                __FILE__, __LINE__, _a, _b, (uint16_t)(tol)); \
        assert(0); \
    } \
} while(0)

/* Standard test config: all servos 1000-1500-2000, not reversed except right side */
static vms_mixer_config_t test_config(void)
{
    vms_servo_config_t std_servo   = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 0 };
    vms_servo_config_t std_rev     = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 1 };
    vms_servo_config_t std_esc     = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 0 };

    vms_mixer_config_t cfg;
    cfg.motor_left    = std_esc;
    cfg.motor_right   = std_esc;
    cfg.tilt_left     = std_servo;
    cfg.tilt_right    = std_rev;
    cfg.aileron_left  = std_servo;
    cfg.aileron_right = std_rev;
    cfg.flap_left     = std_servo;
    cfg.flap_right    = std_rev;
    cfg.rudder        = std_servo;
    return cfg;
}

static void test_disarmed(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    vms_mixer_output_t out = vms_mixer_disarmed();

    /* Motors off (min) */
    ASSERT_EQ(out.channels[ACT_MOTOR_LEFT],  1000);
    ASSERT_EQ(out.channels[ACT_MOTOR_RIGHT], 1000);
    /* Servos centered */
    ASSERT_EQ(out.channels[ACT_TILT_LEFT],     1500);
    ASSERT_EQ(out.channels[ACT_TILT_RIGHT],    1500);
    ASSERT_EQ(out.channels[ACT_AILERON_LEFT],  1500);
    ASSERT_EQ(out.channels[ACT_AILERON_RIGHT], 1500);
    ASSERT_EQ(out.channels[ACT_RUDDER],        1500);

    printf("  PASS: disarmed\n");
}

static void test_hover_neutral(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    vms_mixer_input_t in = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 0.0f, .flap = 0.0f
    };
    vms_mixer_output_t out = vms_mixer_hover(&in);

    /* Both motors at 50% throttle: 1000 + 0.5*1000 = 1500 */
    ASSERT_EQ(out.channels[ACT_MOTOR_LEFT],  1500);
    ASSERT_EQ(out.channels[ACT_MOTOR_RIGHT], 1500);
    /* Tilts centered (0 pitch) */
    ASSERT_EQ(out.channels[ACT_TILT_LEFT],  1500);
    /* Right tilt reversed: center stays at 1500 for 0 input */
    ASSERT_EQ(out.channels[ACT_TILT_RIGHT], 1500);
    /* Surfaces centered */
    ASSERT_EQ(out.channels[ACT_AILERON_LEFT],  1500);
    ASSERT_EQ(out.channels[ACT_AILERON_RIGHT], 1500);
    ASSERT_EQ(out.channels[ACT_RUDDER],        1500);

    printf("  PASS: hover neutral\n");
}

static void test_hover_roll(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    vms_mixer_input_t in = {
        .throttle = 0.5f,
        .roll = 1.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 0.0f, .flap = 0.0f
    };
    vms_mixer_output_t out = vms_mixer_hover(&in);

    /* Roll right: left motor higher, right motor lower */
    /* Left: 0.5 + 1.0*0.3 = 0.8 → 1000 + 0.8*1000 = 1800 */
    ASSERT_EQ(out.channels[ACT_MOTOR_LEFT], 1800);
    /* Right: 0.5 - 1.0*0.3 = 0.2 → 1000 + 0.2*1000 = 1200 */
    ASSERT_EQ(out.channels[ACT_MOTOR_RIGHT], 1200);

    printf("  PASS: hover roll\n");
}

static void test_hover_yaw(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    vms_mixer_input_t in = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 1.0f,
        .tilt = 0.0f, .flap = 0.0f
    };
    vms_mixer_output_t out = vms_mixer_hover(&in);

    /* Yaw CW: left motor higher, right lower (differential) */
    /* Left: 0.5 + 0.3 = 0.8 → 1800 */
    ASSERT_EQ(out.channels[ACT_MOTOR_LEFT], 1800);
    /* Right: 0.5 - 0.3 = 0.2 → 1200 */
    ASSERT_EQ(out.channels[ACT_MOTOR_RIGHT], 1200);

    printf("  PASS: hover yaw\n");
}

static void test_cruise_surfaces(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    vms_mixer_input_t in = {
        .throttle = 0.6f,
        .roll = 0.5f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 1.0f, .flap = 0.0f
    };
    vms_mixer_output_t out = vms_mixer_cruise(&in);

    /* Motors equal at 60%: 1000 + 0.6*1000 = 1600 */
    ASSERT_EQ(out.channels[ACT_MOTOR_LEFT],  1600);
    ASSERT_EQ(out.channels[ACT_MOTOR_RIGHT], 1600);

    /* Tilts fully horizontal: tilt_left → map(1.0) = 2000, tilt_right reversed → map(-1.0) = 1000 */
    ASSERT_EQ(out.channels[ACT_TILT_LEFT],  2000);
    ASSERT_EQ(out.channels[ACT_TILT_RIGHT], 1000);

    /* Aileron left: 0.5 roll, no pitch → map(0.5) = 1750 */
    ASSERT_EQ(out.channels[ACT_AILERON_LEFT], 1750);
    /* Aileron right: reversed, map(0.5) → maps -0.5 → 1250 */
    ASSERT_EQ(out.channels[ACT_AILERON_RIGHT], 1250);

    printf("  PASS: cruise surfaces\n");
}

static void test_transition_blend(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    /* At tilt=0.0 (full hover), transition should equal hover */
    vms_mixer_input_t in_hover = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 0.0f, .flap = 0.0f
    };
    vms_mixer_output_t t_out = vms_mixer_transition(&in_hover);
    vms_mixer_output_t h_out = vms_mixer_hover(&in_hover);

    /* Motor outputs should match hover at tilt=0 */
    ASSERT_EQ(t_out.channels[ACT_MOTOR_LEFT],  h_out.channels[ACT_MOTOR_LEFT]);
    ASSERT_EQ(t_out.channels[ACT_MOTOR_RIGHT], h_out.channels[ACT_MOTOR_RIGHT]);

    /* At tilt=1.0 (full cruise), transition should equal cruise */
    vms_mixer_input_t in_cruise = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 1.0f, .flap = 0.0f
    };
    t_out = vms_mixer_transition(&in_cruise);
    vms_mixer_output_t c_out = vms_mixer_cruise(&in_cruise);

    ASSERT_EQ(t_out.channels[ACT_MOTOR_LEFT],  c_out.channels[ACT_MOTOR_LEFT]);
    ASSERT_EQ(t_out.channels[ACT_MOTOR_RIGHT], c_out.channels[ACT_MOTOR_RIGHT]);

    /* At tilt=0.5 (mid transition), motors should be average of hover and cruise */
    vms_mixer_input_t in_mid = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 0.5f, .flap = 0.0f
    };
    t_out = vms_mixer_transition(&in_mid);
    h_out = vms_mixer_hover(&in_mid);
    c_out = vms_mixer_cruise(&in_mid);

    uint16_t expected_motor_l = (uint16_t)((h_out.channels[ACT_MOTOR_LEFT]
                                           + c_out.channels[ACT_MOTOR_LEFT]) / 2);
    ASSERT_NEAR_U16(t_out.channels[ACT_MOTOR_LEFT], expected_motor_l, 1);

    printf("  PASS: transition blend\n");
}

static void test_motor_clamping(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    /* Full throttle + full roll + full yaw should not exceed ESC max */
    vms_mixer_input_t in = {
        .throttle = 1.0f,
        .roll = 1.0f, .pitch = 0.0f, .yaw = 1.0f,
        .tilt = 0.0f, .flap = 0.0f
    };
    vms_mixer_output_t out = vms_mixer_hover(&in);

    /* Clamped to [1000, 2000] */
    assert(out.channels[ACT_MOTOR_LEFT]  <= 2000);
    assert(out.channels[ACT_MOTOR_LEFT]  >= 1000);
    assert(out.channels[ACT_MOTOR_RIGHT] <= 2000);
    assert(out.channels[ACT_MOTOR_RIGHT] >= 1000);

    printf("  PASS: motor clamping\n");
}

static void test_flaps(void)
{
    vms_mixer_config_t cfg = test_config();
    vms_mixer_init(&cfg);

    /* Flaps deployed */
    vms_mixer_input_t in = {
        .throttle = 0.5f,
        .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
        .tilt = 0.0f, .flap = 1.0f
    };
    vms_mixer_output_t out = vms_mixer_hover(&in);

    /* Flap left: map(1.0) = 2000 */
    ASSERT_EQ(out.channels[ACT_FLAP_LEFT], 2000);
    /* Flap right: reversed, map(1.0) → map(-1.0) = 1000 */
    ASSERT_EQ(out.channels[ACT_FLAP_RIGHT], 1000);

    printf("  PASS: flaps\n");
}

int main(void)
{
    printf("Running mixer tests...\n");
    test_disarmed();
    test_hover_neutral();
    test_hover_roll();
    test_hover_yaw();
    test_cruise_surfaces();
    test_transition_blend();
    test_motor_clamping();
    test_flaps();
    printf("All mixer tests passed.\n");
    return 0;
}
