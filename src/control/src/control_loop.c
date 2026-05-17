#define _GNU_SOURCE
#include "control_loop.h"
#include "pid.h"
#include "mixer.h"
#include "attitude.h"
#include "bno085.h"
#include "pwm_output.h"
#include "rc_input.h"
#include "gpio.h"

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>

#define LOOP_RATE_HZ    400
#define LOOP_PERIOD_NS  (1000000000L / LOOP_RATE_HZ)  /* 2,500,000 ns = 2.5ms */
#define RT_CPU_CORE     3
#define RT_PRIORITY     80

static pthread_t g_thread;
static atomic_int g_running = 0;
static vms_hal_config_t g_hal_cfg;

static void timespec_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

static long timespec_diff_us(const struct timespec *a, const struct timespec *b)
{
    return (a->tv_sec - b->tv_sec) * 1000000L
         + (a->tv_nsec - b->tv_nsec) / 1000L;
}

static void set_rt_priority(void)
{
    /* Pin to isolated CPU core */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RT_CPU_CORE, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    /* Set SCHED_FIFO real-time priority */
    struct sched_param param;
    param.sched_priority = RT_PRIORITY;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
}

static void *control_thread(void *arg)
{
    (void)arg;
    vms_shared_state_t *shm = vms_shm_get();

    set_rt_priority();

    /* ── Initialize HAL ─────────────────────────────────────────── */
    if (vms_gpio_init() != 0) {
        fprintf(stderr, "control: gpio init failed\n");
        atomic_store(&g_running, 0);
        return NULL;
    }
    if (vms_bno085_init(g_hal_cfg.i2c_bus, g_hal_cfg.bno085_addr, LOOP_RATE_HZ) != 0) {
        fprintf(stderr, "control: IMU init failed\n");
        vms_gpio_close();
        atomic_store(&g_running, 0);
        return NULL;
    }
    if (vms_pwm_init(g_hal_cfg.i2c_bus, g_hal_cfg.pca9685_addr) != 0) {
        fprintf(stderr, "control: PWM init failed\n");
        vms_bno085_close();
        vms_gpio_close();
        atomic_store(&g_running, 0);
        return NULL;
    }
    if (vms_rc_init(g_hal_cfg.rc_pins, g_hal_cfg.rc_pin_count) != 0) {
        fprintf(stderr, "control: RC init failed\n");
        vms_pwm_close();
        vms_bno085_close();
        vms_gpio_close();
        atomic_store(&g_running, 0);
        return NULL;
    }

    /* ── Initialize PID controllers ─────────────────────────────── */
    vms_pid_t pid_hover[VMS_PID_AXES];
    vms_pid_t pid_cruise[VMS_PID_AXES];

    for (int i = 0; i < VMS_PID_AXES; i++) {
        vms_pid_init(&pid_hover[i],  &shm->pid_gains_hover[i]);
        vms_pid_init(&pid_cruise[i], &shm->pid_gains_cruise[i]);
    }

    /* ── Initialize mixer with default config ───────────────────── */
    vms_mixer_config_t mix_cfg;
    vms_servo_config_t esc_default   = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 0 };
    vms_servo_config_t servo_default = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 0 };
    vms_servo_config_t servo_rev     = { .min_us = 1000, .center_us = 1500, .max_us = 2000, .reversed = 1 };

    mix_cfg.motor_left    = esc_default;
    mix_cfg.motor_right   = esc_default;
    mix_cfg.tilt_left     = servo_default;
    mix_cfg.tilt_right    = servo_rev;
    mix_cfg.aileron_left  = servo_default;
    mix_cfg.aileron_right = servo_rev;
    mix_cfg.flap_left     = servo_default;
    mix_cfg.flap_right    = servo_rev;
    mix_cfg.rudder        = servo_default;
    vms_mixer_init(&mix_cfg);

    /* ── Main loop ──────────────────────────────────────────────── */
    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    while (atomic_load(&g_running)) {
        struct timespec loop_start;
        clock_gettime(CLOCK_MONOTONIC, &loop_start);

        /* 1. Read IMU */
        vms_imu_data_t imu;
        if (vms_bno085_read(&imu) == 0) {
            shm->quaternion[0] = imu.w;
            shm->quaternion[1] = imu.x;
            shm->quaternion[2] = imu.y;
            shm->quaternion[3] = imu.z;

            /* Convert gyro from rad/s to deg/s */
            shm->gyro_dps[0] = imu.gyro_x * 57.29577951f;
            shm->gyro_dps[1] = imu.gyro_y * 57.29577951f;
            shm->gyro_dps[2] = imu.gyro_z * 57.29577951f;

            /* 2. Compute Euler angles */
            vms_euler_t euler = vms_quat_to_euler(imu.w, imu.x, imu.y, imu.z);
            shm->euler_deg[0] = euler.roll_deg;
            shm->euler_deg[1] = euler.pitch_deg;
            shm->euler_deg[2] = euler.yaw_deg;
        }

        /* 3. Read RC */
        int rc_ok = vms_rc_read(shm->rc_channels, VMS_RC_CHANNELS);
        atomic_store(&shm->rc_valid, rc_ok ? 1 : 0);

        /* 4. Check for PID gain hot-reload */
        if (atomic_exchange(&shm->pid_gains_updated, 0)) {
            for (int i = 0; i < VMS_PID_AXES; i++) {
                vms_pid_set_gains(&pid_hover[i],  &shm->pid_gains_hover[i]);
                vms_pid_set_gains(&pid_cruise[i], &shm->pid_gains_cruise[i]);
            }
        }

        /* 5. Run PID + mixer based on flight mode */
        int32_t mode = atomic_load(&shm->flight_mode);
        float tilt_cmd = atomic_load(&shm->tilt_command);
        float flap_cmd = atomic_load(&shm->flap_command);
        uint8_t safety = atomic_load(&shm->safety_override);

        float dt = 1.0f / LOOP_RATE_HZ;
        vms_mixer_output_t mix_out;

        if (mode == MODE_DISARMED || safety) {
            /* Motors off, servos centered */
            mix_out = vms_mixer_disarmed();
            for (int i = 0; i < VMS_PID_AXES; i++) {
                vms_pid_reset(&pid_hover[i]);
                vms_pid_reset(&pid_cruise[i]);
            }
            shm->pid_output[0] = 0.0f;
            shm->pid_output[1] = 0.0f;
            shm->pid_output[2] = 0.0f;
        } else {
            /* Compute PID — setpoint is 0 (level flight) for now.
             * Future: setpoint comes from RC stick position. */
            float setpoint_roll  = 0.0f;
            float setpoint_pitch = 0.0f;
            float setpoint_yaw   = 0.0f;

            /* Select PID bank based on mode */
            vms_pid_t *active_pid = (mode == MODE_FIXED_WING_CRUISE)
                                  ? pid_cruise : pid_hover;

            shm->pid_output[0] = vms_pid_update(&active_pid[0],
                                    setpoint_roll,  shm->euler_deg[0], dt);
            shm->pid_output[1] = vms_pid_update(&active_pid[1],
                                    setpoint_pitch, shm->euler_deg[1], dt);
            shm->pid_output[2] = vms_pid_update(&active_pid[2],
                                    setpoint_yaw,   shm->euler_deg[2], dt);

            /* Build mixer input */
            vms_mixer_input_t mix_in = {
                .throttle = 0.0f, /* Will be mapped from RC CH3 */
                .roll  = shm->pid_output[0],
                .pitch = shm->pid_output[1],
                .yaw   = shm->pid_output[2],
                .tilt  = tilt_cmd,
                .flap  = flap_cmd,
            };

            /* Map RC throttle (CH3: 1000-2000 → 0.0-1.0) */
            if (shm->rc_channels[2] >= 1000 && shm->rc_channels[2] <= 2000) {
                mix_in.throttle = (float)(shm->rc_channels[2] - 1000) / 1000.0f;
            }

            /* Select mixer based on mode */
            switch (mode) {
                case MODE_VTOL_HOVER:
                    mix_out = vms_mixer_hover(&mix_in);
                    break;
                case MODE_FIXED_WING_CRUISE:
                    mix_out = vms_mixer_cruise(&mix_in);
                    break;
                case MODE_TRANSITIONING:
                case MODE_TRANSITIONING_BACK:
                    mix_out = vms_mixer_transition(&mix_in);
                    break;
                case MODE_FAILSAFE:
                    /* Failsafe: low throttle hover */
                    mix_in.throttle = 0.3f;
                    mix_in.tilt = 0.0f;
                    mix_out = vms_mixer_hover(&mix_in);
                    break;
                default:
                    mix_out = vms_mixer_disarmed();
                    break;
            }
        }

        /* 6. Write actuator outputs */
        memcpy(shm->actuator_us, mix_out.channels, sizeof(shm->actuator_us));

        /* Write to PCA9685 using configured channel mapping */
        for (int i = 0; i < VMS_NUM_ACTUATORS; i++) {
            vms_pwm_set(g_hal_cfg.pwm_channels[i], mix_out.channels[i]);
        }

        /* 7. Update timing stats */
        struct timespec loop_end;
        clock_gettime(CLOCK_MONOTONIC, &loop_end);
        atomic_store(&shm->loop_dt_us,
                     (uint32_t)timespec_diff_us(&loop_end, &loop_start));
        atomic_fetch_add(&shm->loop_counter, 1);

        /* 8. Sleep until next period */
        timespec_add_ns(&next_wake, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, NULL);
    }

    /* ── Shutdown ───────────────────────────────────────────────── */
    vms_pwm_all_off();
    vms_rc_close();
    vms_pwm_close();
    vms_bno085_close();
    vms_gpio_close();

    return NULL;
}

int vms_control_loop_start(const vms_hal_config_t *hal_cfg)
{
    if (atomic_load(&g_running))
        return -1;  /* Already running */

    g_hal_cfg = *hal_cfg;
    atomic_store(&g_running, 1);

    if (pthread_create(&g_thread, NULL, control_thread, NULL) != 0) {
        atomic_store(&g_running, 0);
        return -1;
    }

    return 0;
}

void vms_control_loop_stop(void)
{
    if (!atomic_load(&g_running))
        return;

    atomic_store(&g_running, 0);
    pthread_join(g_thread, NULL);
}

int vms_control_loop_is_running(void)
{
    return atomic_load(&g_running);
}
