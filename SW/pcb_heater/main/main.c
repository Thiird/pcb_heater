#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_types.h"

#define ACTIVITY_LED 4 // GPIO4, active highssss

void adjust_temp()
{
    while (true)
    {
    }
}

esp_err_t init_gpio()
{
    // Activity LED
    gpio_reset_pin(ACTIVITY_LED);
    gpio_set_direction(ACTIVITY_LED, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

esp_err_t init_uart()
{
    // Configure UART parameters
    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO39, RX: IO44, RTS: IO5, CTS: IO6), RTS and CTS not used
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 39, 44, 5, 6));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));
    return ESP_OK;
}

void app_main(void)
{
    init_gpio();
    init_uart();

    char test_str[32];
    static uint8_t led_state = 0;
    int cont = 0;

    while (true)
    {
        // Write data to UART.
        snprintf(&test_str, 32, "Test string %d.\n", cont);
        uart_write_bytes(UART_NUM_0, (const char *)test_str, 32);

        gpio_set_level(ACTIVITY_LED, led_state);
        led_state = !led_state;

        cont++;
        // 1 sec delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // it's cold and it's getting dark...
}
