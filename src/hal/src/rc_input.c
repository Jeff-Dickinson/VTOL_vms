#include "rc_input.h"
#include <pigpio.h>
#include <string.h>
#include <time.h>

/*
 * RC PWM pulse measurement via pigpio callbacks.
 *
 * Each RC channel outputs a PWM signal (1000-2000μs pulse, 50Hz).
 * We use pigpio's alertFunc callbacks to measure rising→falling edge times
 * with hardware-level timing accuracy.
 *
 * Signal loss is detected by checking the time since the last valid pulse.
 */

#define PULSE_MIN_US    800     /* Reject glitches below this */
#define PULSE_MAX_US    2200    /* Reject glitches above this */
#define SIGNAL_TIMEOUT_MS 500   /* Signal lost after this */

static int g_pins[VMS_RC_MAX_CHANNELS];
static int g_num_pins = 0;
static volatile uint16_t g_channels[VMS_RC_MAX_CHANNELS];
static volatile uint32_t g_rise_tick[VMS_RC_MAX_CHANNELS];
static volatile uint32_t g_last_valid_tick;

static void rc_callback(int gpio, int level, uint32_t tick)
{
    /* Find which channel this GPIO belongs to */
    int ch = -1;
    for (int i = 0; i < g_num_pins; i++) {
        if (g_pins[i] == gpio) {
            ch = i;
            break;
        }
    }
    if (ch < 0) return;

    if (level == 1) {
        /* Rising edge: record start time */
        g_rise_tick[ch] = tick;
    } else if (level == 0) {
        /* Falling edge: compute pulse width */
        uint32_t pulse = tick - g_rise_tick[ch];
        if (pulse >= PULSE_MIN_US && pulse <= PULSE_MAX_US) {
            g_channels[ch] = (uint16_t)pulse;
            g_last_valid_tick = tick;
        }
    }
}

int vms_rc_init(const int *gpio_pins, int num_pins)
{
    if (num_pins > VMS_RC_MAX_CHANNELS)
        num_pins = VMS_RC_MAX_CHANNELS;

    memset((void *)g_channels, 0, sizeof(g_channels));
    memset((void *)g_rise_tick, 0, sizeof(g_rise_tick));
    g_last_valid_tick = gpioTick();
    g_num_pins = num_pins;

    for (int i = 0; i < num_pins; i++) {
        g_pins[i] = gpio_pins[i];
        gpioSetMode(gpio_pins[i], PI_INPUT);
        gpioSetPullUpDown(gpio_pins[i], PI_PUD_DOWN);
        if (gpioSetAlertFunc(gpio_pins[i], rc_callback) != 0) {
            return -1;
        }
    }

    return 0;
}

int vms_rc_read(uint16_t *channels_out, int count)
{
    if (count > g_num_pins) count = g_num_pins;

    for (int i = 0; i < count; i++) {
        channels_out[i] = g_channels[i];
    }

    uint32_t now = gpioTick();
    uint32_t elapsed_us = now - g_last_valid_tick;
    return (elapsed_us < SIGNAL_TIMEOUT_MS * 1000) ? 1 : 0;
}

uint32_t vms_rc_ms_since_last(void)
{
    uint32_t now = gpioTick();
    return (now - g_last_valid_tick) / 1000;
}

void vms_rc_close(void)
{
    for (int i = 0; i < g_num_pins; i++) {
        gpioSetAlertFunc(g_pins[i], NULL);
    }
    g_num_pins = 0;
}
