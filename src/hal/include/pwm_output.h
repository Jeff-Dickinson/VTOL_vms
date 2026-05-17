#ifndef VMS_PWM_OUTPUT_H
#define VMS_PWM_OUTPUT_H

#include <stdint.h>

/**
 * PCA9685 16-channel PWM driver over I2C.
 *
 * Generates servo/ESC PWM signals at 50Hz (20ms period).
 * Each channel can output 1000-2000μs pulses independently.
 *
 * Hardware: PCA9685 on I2C bus 1, address 0x40.
 */

#define VMS_PWM_CHANNELS 16

/**
 * Initialize PCA9685 on specified I2C bus and address.
 * Sets PWM frequency to ~50Hz for servo/ESC compatibility.
 *
 * @param i2c_bus   I2C bus number (typically 1)
 * @param i2c_addr  I2C address (typically 0x40)
 * @return 0 on success, -1 on failure
 */
int vms_pwm_init(int i2c_bus, int i2c_addr);

/**
 * Set a single channel's pulse width in microseconds.
 *
 * @param channel  Channel number (0-15)
 * @param pulse_us Pulse width in microseconds (typically 1000-2000)
 * @return 0 on success, -1 on invalid channel
 */
int vms_pwm_set(int channel, uint16_t pulse_us);

/**
 * Set multiple channels atomically (writes all in one I2C transaction).
 *
 * @param values   Array of pulse widths (μs), indexed by channel
 * @param count    Number of channels to write (starting from channel 0)
 * @return 0 on success, -1 on failure
 */
int vms_pwm_set_multi(const uint16_t *values, int count);

/**
 * Turn off all PWM outputs (set all channels to 0).
 */
void vms_pwm_all_off(void);

/**
 * Shut down PCA9685 communication.
 */
void vms_pwm_close(void);

#endif /* VMS_PWM_OUTPUT_H */
