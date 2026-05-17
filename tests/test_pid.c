#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "pid.h"

#define ASSERT_NEAR(a, b, tol) do { \
    float _a = (a), _b = (b); \
    if (fabsf(_a - _b) > (tol)) { \
        fprintf(stderr, "FAIL %s:%d: %.6f != %.6f (tol %.6f)\n", \
                __FILE__, __LINE__, _a, _b, (float)(tol)); \
        assert(0); \
    } \
} while(0)

static void test_proportional_only(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 2.0f, .ki = 0.0f, .kd = 0.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 1.0f
    };
    vms_pid_init(&pid, &gains);

    /* error = 5.0 - 3.0 = 2.0, P = 2.0 * 2.0 = 4.0 */
    float out = vms_pid_update(&pid, 5.0f, 3.0f, 0.01f);
    ASSERT_NEAR(out, 4.0f, 0.001f);

    printf("  PASS: proportional only\n");
}

static void test_integral_accumulation(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 0.0f, .ki = 1.0f, .kd = 0.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 10.0f
    };
    vms_pid_init(&pid, &gains);

    float dt = 0.01f;
    /* error = 1.0 each step, integral accumulates 1.0*0.01 = 0.01 per step */
    for (int i = 0; i < 100; i++) {
        vms_pid_update(&pid, 1.0f, 0.0f, dt);
    }
    /* integral = 100 * 1.0 * 0.01 = 1.0, I_term = 1.0 * 1.0 = 1.0 */
    float out = vms_pid_update(&pid, 1.0f, 0.0f, dt);
    /* After 101 steps: integral = 1.01, output = 1.01 */
    ASSERT_NEAR(out, 1.01f, 0.001f);

    printf("  PASS: integral accumulation\n");
}

static void test_anti_windup(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 0.0f, .ki = 1.0f, .kd = 0.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 0.5f
    };
    vms_pid_init(&pid, &gains);

    /* Run many steps with constant error — integral should clamp at 0.5 */
    for (int i = 0; i < 1000; i++) {
        vms_pid_update(&pid, 10.0f, 0.0f, 0.01f);
    }
    /* integral clamped to 0.5, output = ki * integral = 0.5 */
    float out = vms_pid_update(&pid, 10.0f, 0.0f, 0.01f);
    ASSERT_NEAR(out, 0.5f, 0.001f);

    printf("  PASS: anti-windup\n");
}

static void test_derivative_on_measurement(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 0.0f, .ki = 0.0f, .kd = 1.0f,
        .output_min = -100.0f, .output_max = 100.0f, .integral_max = 1.0f
    };
    vms_pid_init(&pid, &gains);

    float dt = 0.01f;

    /* First call: no derivative (not initialized) */
    float out1 = vms_pid_update(&pid, 0.0f, 0.0f, dt);
    ASSERT_NEAR(out1, 0.0f, 0.001f);

    /* Second call: measurement jumps from 0 to 1.
     * d_measurement = (1.0 - 0.0) / 0.01 = 100.0
     * d_term = -kd * d_measurement = -1.0 * 100.0 = -100.0 */
    float out2 = vms_pid_update(&pid, 0.0f, 1.0f, dt);
    ASSERT_NEAR(out2, -100.0f, 0.001f);

    /* Third call: measurement steady at 1.0, d_measurement = 0 */
    float out3 = vms_pid_update(&pid, 0.0f, 1.0f, dt);
    ASSERT_NEAR(out3, 0.0f, 0.001f);

    printf("  PASS: derivative on measurement\n");
}

static void test_output_clamping(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 10.0f, .ki = 0.0f, .kd = 0.0f,
        .output_min = -1.0f, .output_max = 1.0f, .integral_max = 1.0f
    };
    vms_pid_init(&pid, &gains);

    /* Large error: kp*error = 10*5 = 50, should clamp to 1.0 */
    float out_pos = vms_pid_update(&pid, 5.0f, 0.0f, 0.01f);
    ASSERT_NEAR(out_pos, 1.0f, 0.001f);

    /* Negative error: kp*error = 10*(-5) = -50, should clamp to -1.0 */
    float out_neg = vms_pid_update(&pid, -5.0f, 0.0f, 0.01f);
    ASSERT_NEAR(out_neg, -1.0f, 0.001f);

    printf("  PASS: output clamping\n");
}

static void test_reset(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 1.0f, .ki = 1.0f, .kd = 1.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 10.0f
    };
    vms_pid_init(&pid, &gains);

    /* Accumulate some state */
    for (int i = 0; i < 50; i++) {
        vms_pid_update(&pid, 1.0f, 0.0f, 0.01f);
    }
    assert(pid.integral > 0.0f);
    assert(pid.initialized == 1);

    /* Reset should zero everything */
    vms_pid_reset(&pid);
    assert(pid.integral == 0.0f);
    assert(pid.prev_measurement == 0.0f);
    assert(pid.initialized == 0);

    printf("  PASS: reset\n");
}

static void test_set_gains(void)
{
    vms_pid_t pid;
    vms_pid_gains_t gains = {
        .kp = 1.0f, .ki = 1.0f, .kd = 0.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 5.0f
    };
    vms_pid_init(&pid, &gains);

    /* Build up integral */
    for (int i = 0; i < 100; i++) {
        vms_pid_update(&pid, 1.0f, 0.0f, 0.01f);
    }
    /* integral = 1.0 */

    /* Hot-reload with smaller integral_max */
    vms_pid_gains_t new_gains = {
        .kp = 2.0f, .ki = 1.0f, .kd = 0.0f,
        .output_min = -10.0f, .output_max = 10.0f, .integral_max = 0.5f
    };
    vms_pid_set_gains(&pid, &new_gains);

    /* Integral should be clamped to 0.5 */
    ASSERT_NEAR(pid.integral, 0.5f, 0.001f);
    /* kp should be updated */
    ASSERT_NEAR(pid.kp, 2.0f, 0.001f);

    printf("  PASS: set_gains hot reload\n");
}

int main(void)
{
    printf("Running PID tests...\n");
    test_proportional_only();
    test_integral_accumulation();
    test_anti_windup();
    test_derivative_on_measurement();
    test_output_clamping();
    test_reset();
    test_set_gains();
    printf("All PID tests passed.\n");
    return 0;
}
