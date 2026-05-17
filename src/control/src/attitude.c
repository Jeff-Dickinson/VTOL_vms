#include "attitude.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

vms_euler_t vms_quat_to_euler(float w, float x, float y, float z)
{
    vms_euler_t e;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    e.roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    /* Pitch (y-axis rotation) — clamp to avoid NaN at gimbal lock */
    float sinp = 2.0f * (w * y - z * x);
    if (sinp >= 1.0f)
        e.pitch_deg = 90.0f;
    else if (sinp <= -1.0f)
        e.pitch_deg = -90.0f;
    else
        e.pitch_deg = asinf(sinp) * RAD_TO_DEG;

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    e.yaw_deg = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return e;
}

int vms_quat_normalize(float *w, float *x, float *y, float *z)
{
    float mag = sqrtf((*w)*(*w) + (*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    if (mag < 1e-8f)
        return -1;

    float inv = 1.0f / mag;
    *w *= inv;
    *x *= inv;
    *y *= inv;
    *z *= inv;
    return 0;
}
