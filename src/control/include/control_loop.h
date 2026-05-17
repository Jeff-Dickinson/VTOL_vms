#ifndef VMS_CONTROL_LOOP_H
#define VMS_CONTROL_LOOP_H

#include "shm_interface.h"

/**
 * 400Hz real-time control loop.
 *
 * Runs on an isolated CPU core (core 3) with SCHED_FIFO priority.
 * Each iteration: read IMU → compute attitude → PID → mixer → write PWM.
 *
 * Spawned from Python via ctypes. Communicates through shared memory only.
 */

/**
 * HAL configuration passed from Python at startup.
 */
typedef struct {
    /* I2C */
    int i2c_bus;
    int bno085_addr;
    int pca9685_addr;
    /* RC GPIO pins (BCM) */
    int rc_pins[8];
    int rc_pin_count;
    /* PWM channel mapping (PCA9685 channel for each actuator) */
    int pwm_channels[VMS_NUM_ACTUATORS];
} vms_hal_config_t;

/**
 * Start the real-time control loop in a new thread.
 *
 * This function:
 *   1. Initializes HAL (GPIO/pigpio, IMU, PCA9685, RC input)
 *   2. Sets SCHED_FIFO priority and pins to the specified CPU core
 *   3. Enters the 400Hz loop
 *
 * The thread runs until vms_control_loop_stop() is called.
 *
 * @param hal_cfg   Hardware configuration
 * @return 0 on success (thread started), -1 on failure
 */
int vms_control_loop_start(const vms_hal_config_t *hal_cfg);

/**
 * Signal the control loop to stop and wait for the thread to exit.
 * Also shuts down all HAL components and sets motors to off.
 */
void vms_control_loop_stop(void);

/**
 * Check if the control loop is currently running.
 * @return 1 if running, 0 if stopped
 */
int vms_control_loop_is_running(void);

#endif /* VMS_CONTROL_LOOP_H */
