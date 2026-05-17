#ifndef VMS_GPIO_H
#define VMS_GPIO_H

/**
 * GPIO utility functions.
 * Thin wrapper around pigpio for initialization and cleanup.
 */

/**
 * Initialize pigpio library.
 * Must be called before any HAL component that uses GPIO.
 *
 * @return 0 on success, -1 on failure
 */
int vms_gpio_init(void);

/**
 * Terminate pigpio library.
 * Call on shutdown after all HAL components are closed.
 */
void vms_gpio_close(void);

#endif /* VMS_GPIO_H */
