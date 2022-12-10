#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "hal/uart_types.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "../include/i2c.h"

#define ACTIVITY_LED 4        // GPIO4, activity led, active high
#define TEMP_INC_BTN 7        // GPIO7, increase target temperature, active low
#define TEMP_DEC_BTN 8        // GPIO8, decrease target temperature, active low
#define START_STOP_BTN 9      // GPIO9, start/stop heating, active low
#define HEATER_MOSFET_GATE 38 // GPIO38
#define MAX_ALLOWED_TEMP 55   // °C
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQ 1000 // 1KHz

uint8_t heater_current_I = 0;
uint8_t heater_current_temp_F = 0;
float heater_current_temp_C = 0;
uint8_t heater_target_temp = 20; // °C
bool is_heating_active = false;
uint8_t mosfet_duty_cycle = 0;

adc_oneshot_unit_handle_t adc2_handle = NULL;
uint16_t adcTempVal = 0;
uint16_t adcIVal = 0;

void update_oled()
{
}

void update_duty_cycle()
{
    // duty cycle formula is ((2 ^ bit_res) - 1) * desired_duty_cycle = 4095

    // Set new duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, ((2 ^ LEDC_DUTY_RES) - 1) * mosfet_duty_cycle));
    // Update duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void read_heater_current()
{
    // I = V / R

    adc_oneshot_read(&adc2_handle, ADC_CHANNEL_3, &adcIVal);

    heater_current_I = 0; // TODO
}

void read_heater_temp()
{
    // Reference: https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html

    adc_oneshot_read(&adc2_handle, ADC_CHANNEL_3, &adcTempVal);
    // NTC thermistor config: +12V---NTC---10kOhm---GND
    // Usual v. divider equation is Vout = Vs * (R0 / ( Rt + R0 ))
    // Rearrange to find NTC resistance: Rt = R0 * (( Vs / Vout ) - 1)
    // In our case the ADC's Vref is equal to the Vin of the V. divider
    // so we can say: adcMax / adcTempVal = Vref / Vout
    // With this we can simplify to Rt = R0 * ((adcMax / adcTempVal) - 1)
    // ADC on ESP32-S2 is 12-bit, so adcMax is 4095
    heater_current_temp_F = 10000 * ((4095 / adcTempVal) - 1);

    // Fahrenheit --> Celsius
    heater_current_temp_C = (heater_current_temp_F - 32) / 1.8;
}

void isr_change_heater_target_temp(uint8_t value)
{
    if (value)
    {
        mosfet_duty_cycle++;
        if (heater_target_temp < MAX_ALLOWED_TEMP)
        {
            heater_target_temp++;
        }
        return;
    }

    mosfet_duty_cycle--;

    if (mosfet_duty_cycle > 100)
    {
        mosfet_duty_cycle = 100;
    }

    if (mosfet_duty_cycle < 0)
    {
        mosfet_duty_cycle = 0;
    }

    if (heater_target_temp > 0)
    {
        heater_target_temp--;
    }
}

void isr_start_stop_heating()
{
    is_heating_active = !is_heating_active;
}

esp_err_t init_adc()
{
    // Configure ADC2 instance
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_handle));

    // Create config for ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, // can't use 13 bits, bit 1 always 0, see errata
        .atten = ADC_ATTEN_DB_11,
    };

    // Apply config to Channel0-ADC2, mosfet temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_0, &config));

    // Apply config to Channel3-ADC3, room temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_3, &config));

    // Apply config to Channel4-ADC2, heater temp (middle unit)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config));

    return ESP_OK;
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

esp_err_t init_pwm()
{
    // Configure LEDC PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LEDC_FREQ,
        .duty_resolution = LEDC_DUTY_RES,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC PWM channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = HEATER_MOSFET_GATE,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
    init_uart();
    init_gpio();
    init_i2c();
    init_adc();
    init_pwm();

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
