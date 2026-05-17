#include "gpio.h"
#include "bno085.h"
#include "pwm_output.h"
#include "rc_input.h"
#include <pigpio.h>
#include <stdio.h>

int vms_gpio_init(void)
{
    if (gpioInitialise() < 0) {
        fprintf(stderr, "hal: pigpio initialization failed\n");
        return -1;
    }
    return 0;
}

void vms_gpio_close(void)
{
    gpioTerminate();
}
