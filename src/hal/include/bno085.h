#ifndef VMS_BNO085_H
#define VMS_BNO085_H

#include <stdint.h>

/**
 * BNO085 IMU driver — SHTP over I2C.
 *
 * Configured for 400Hz rotation vector (quaternion) output.
 * Also provides calibrated gyroscope at the same rate.
 *
 * Hardware: BNO085 on I2C bus 1, address 0x4A.
 */

typedef struct {
    float w, x, y, z;       /* Rotation vector quaternion */
    float gyro_x, gyro_y, gyro_z;  /* Calibrated gyro (rad/s) */
    uint8_t accuracy;        /* Quaternion accuracy estimate (0-3) */
    uint8_t valid;           /* 1 if data has been received */
} vms_imu_data_t;

/**
 * Initialize BNO085 on specified I2C bus and address.
 * Configures rotation vector + gyro reports at desired rate.
 *
 * @param i2c_bus   I2C bus number (typically 1)
 * @param i2c_addr  I2C address (typically 0x4A)
 * @param rate_hz   Desired report rate (up to 400)
 * @return 0 on success, -1 on failure
 */
int vms_bno085_init(int i2c_bus, int i2c_addr, int rate_hz);

/**
 * Read latest IMU data. Non-blocking: returns most recent sample.
 *
 * @param data  Output structure
 * @return 0 on success, -1 if no new data available
 */
int vms_bno085_read(vms_imu_data_t *data);

/**
 * Shut down BNO085 communication.
 */
void vms_bno085_close(void);

#endif /* VMS_BNO085_H */
