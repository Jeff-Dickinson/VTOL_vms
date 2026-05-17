#ifndef VMS_RC_INPUT_H
#define VMS_RC_INPUT_H

#include <stdint.h>

/**
 * RC PWM input via pigpio edge callbacks.
 *
 * Measures pulse widths on up to 8 GPIO pins simultaneously using
 * pigpio's hardware-timed edge detection (microsecond accuracy).
 *
 * Hardware: FlySky FS-R9B receiver, 8 PWM channels via level shifters
 * (5V → 3.3V) on GPIO pins.
 */

#define VMS_RC_MAX_CHANNELS 8

/**
 * Initialize RC input on specified GPIO pins.
 * Requires pigpio to be initialized first (via hal_init).
 *
 * @param gpio_pins  Array of BCM GPIO pin numbers
 * @param num_pins   Number of pins (max VMS_RC_MAX_CHANNELS)
 * @return 0 on success, -1 on failure
 */
int vms_rc_init(const int *gpio_pins, int num_pins);

/**
 * Read current RC channel values.
 *
 * @param channels_out  Output array (pulse widths in μs, typically 1000-2000)
 * @param count         Number of channels to read
 * @return 1 if signal valid, 0 if signal lost (stale data)
 */
int vms_rc_read(uint16_t *channels_out, int count);

/**
 * Get time since last valid pulse on any channel (milliseconds).
 * Used for failsafe detection.
 */
uint32_t vms_rc_ms_since_last(void);

/**
 * Shut down RC input callbacks.
 */
void vms_rc_close(void);

#endif /* VMS_RC_INPUT_H */
