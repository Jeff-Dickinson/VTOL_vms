#ifndef VMS_ATTITUDE_H
#define VMS_ATTITUDE_H

/**
 * Quaternion to Euler angle conversion.
 *
 * BNO085 outputs rotation quaternion [w, x, y, z].
 * We need Euler angles (roll, pitch, yaw) in degrees for PID control.
 *
 * Convention: aerospace (NED-like, right-hand)
 *   Roll:  rotation about X (forward), positive = right wing down
 *   Pitch: rotation about Y (right),   positive = nose up
 *   Yaw:   rotation about Z (down),    positive = clockwise from above
 */

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} vms_euler_t;

/**
 * Convert quaternion [w, x, y, z] to Euler angles in degrees.
 * Handles gimbal lock near ±90° pitch.
 */
vms_euler_t vms_quat_to_euler(float w, float x, float y, float z);

/**
 * Normalize a quaternion in-place. Returns 0 on success, -1 if magnitude ≈ 0.
 */
int vms_quat_normalize(float *w, float *x, float *y, float *z);

#endif /* VMS_ATTITUDE_H */
