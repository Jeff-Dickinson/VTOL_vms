#include "pwm_output.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/*
 * PCA9685 16-channel PWM driver over I2C.
 *
 * Register map (relevant subset):
 *   0x00: MODE1 — oscillator, auto-increment, sleep
 *   0x01: MODE2 — output config
 *   0x06 + 4*ch: LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H
 *   0xFE: PRE_SCALE — PWM frequency prescaler
 *
 * PWM frequency: f = 25MHz / (4096 * (prescale + 1))
 * For 50Hz: prescale = 25e6 / (4096 * 50) - 1 ≈ 121
 */

#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_PRESCALE    0xFE
#define PCA9685_ALL_OFF_H   0xFD

#define MODE1_SLEEP     0x10
#define MODE1_AI        0x20    /* Auto-increment */
#define MODE1_RESTART   0x80

/* 50Hz PWM: 20ms period, 4096 ticks per period */
#define PWM_FREQ_HZ    50
#define PRESCALE_50HZ   121
#define TICKS_PER_US    (4096.0f / 20000.0f)  /* 0.2048 ticks per μs */

static int g_fd = -1;

static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return (write(g_fd, buf, 2) == 2) ? 0 : -1;
}

static int reg_read(uint8_t reg, uint8_t *val)
{
    if (write(g_fd, &reg, 1) != 1) return -1;
    if (read(g_fd, val, 1) != 1) return -1;
    return 0;
}

int vms_pwm_init(int i2c_bus, int i2c_addr)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus);

    g_fd = open(path, O_RDWR);
    if (g_fd < 0) {
        perror("pca9685: open i2c");
        return -1;
    }
    if (ioctl(g_fd, I2C_SLAVE, i2c_addr) < 0) {
        perror("pca9685: ioctl I2C_SLAVE");
        close(g_fd);
        g_fd = -1;
        return -1;
    }

    /* Enter sleep mode to set prescaler */
    reg_write(PCA9685_MODE1, MODE1_SLEEP);
    usleep(5000);

    /* Set prescaler for 50Hz */
    reg_write(PCA9685_PRESCALE, PRESCALE_50HZ);
    usleep(5000);

    /* Wake up, enable auto-increment */
    reg_write(PCA9685_MODE1, MODE1_AI);
    usleep(5000);

    /* Restart */
    reg_write(PCA9685_MODE1, MODE1_AI | MODE1_RESTART);
    usleep(5000);

    /* All channels off initially */
    vms_pwm_all_off();

    return 0;
}

int vms_pwm_set(int channel, uint16_t pulse_us)
{
    if (channel < 0 || channel >= VMS_PWM_CHANNELS)
        return -1;

    /* Convert microseconds to 12-bit tick count */
    uint16_t on_tick = 0;
    uint16_t off_tick = (uint16_t)(pulse_us * TICKS_PER_US);
    if (off_tick > 4095) off_tick = 4095;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        on_tick & 0xFF,
        (on_tick >> 8) & 0x0F,
        off_tick & 0xFF,
        (off_tick >> 8) & 0x0F
    };
    return (write(g_fd, buf, 5) == 5) ? 0 : -1;
}

int vms_pwm_set_multi(const uint16_t *values, int count)
{
    if (count > VMS_PWM_CHANNELS) count = VMS_PWM_CHANNELS;

    for (int i = 0; i < count; i++) {
        if (vms_pwm_set(i, values[i]) != 0)
            return -1;
    }
    return 0;
}

void vms_pwm_all_off(void)
{
    for (int i = 0; i < VMS_PWM_CHANNELS; i++) {
        vms_pwm_set(i, 0);
    }
}

void vms_pwm_close(void)
{
    if (g_fd >= 0) {
        vms_pwm_all_off();
        close(g_fd);
        g_fd = -1;
    }
}
