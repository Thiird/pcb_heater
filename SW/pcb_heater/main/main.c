#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define ACTIVITY_LED 4 // GPIO4, active highssss

void app_main(void)
{
    static uint8_t led_state = 0;

    gpio_reset_pin(ACTIVITY_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ACTIVITY_LED, GPIO_MODE_OUTPUT);
    int ancora = 0;

    if (ancora)
        while (true)
        {
            gpio_set_level(ACTIVITY_LED, led_state);
            led_state != led_state;
            vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        }
}
