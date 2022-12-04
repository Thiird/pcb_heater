#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_types.h"

#define ACTIVITY_LED 4   // GPIO4, activity led, active high
#define TEMP_INC_BTN 7   // GPIO7, increase target temperature, active low
#define TEMP_DEC_BTN 8   // GPIO8, decrease target temperature, active low
#define START_STOP_BTN 9 // GPIO9, start/stop heating, active low

uint8_t heater_target_temp = 20; // degrees Celsius
bool is_heating_active = false;

void update_oled()
{
}

void read_heater_current()
{
}

void read_heater_temp()
{
}

void isr_change_heater_target_temp(uint8_t value)
{
    value ? heater_target_temp++ : heater_target_temp--;
}

void isr_start_stop_heating()
{
    is_heating_active = !is_heating_active;
}

esp_err_t init_gpio()
{
    // Enable per-GPIO interrupts
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // Activity LED
    gpio_config_t io_conf = {
        io_conf.intr_type = GPIO_INTR_DISABLE, // disable interrup,
        io_conf.mode = GPIO_MODE_OUTPUT,       // set as output mode
        io_conf.pin_bit_mask = ACTIVITY_LED,   // bit mask of the pins that you want to set
        io_conf.pull_down_en = 0,              // disable pull-down mode
        io_conf.pull_up_en = 0};               // disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Increase temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = TEMP_INC_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, isr_change_heater_target_temp, 1));

    // Decrease temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = TEMP_DEC_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, isr_change_heater_target_temp, 0));

    // Start/Stop temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = START_STOP_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, isr_start_stop_heating, NULL));

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
        .rx_flow_ctrl_thresh = 122};
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
        snprintf(test_str, 32, "Test string %d.\n", cont);
        uart_write_bytes(UART_NUM_0, (const char *)test_str, 32);

        gpio_set_level(ACTIVITY_LED, led_state);
        led_state = !led_state;

        cont++;
        // 1 sec delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // xTaskCreate(update_oled, "update_oled", 2048, NULL, 5, NULL);
    // xTaskCreate(read_current, "read_current", 2048, NULL, 10, NULL);
    // xTaskCreate(read_heater_temp, "read_heater_temp", 2048, NULL, 10, NULL);
}
