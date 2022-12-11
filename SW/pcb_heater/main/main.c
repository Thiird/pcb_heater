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
#include "ssd1306.h"

#define ACTIVITY_LED 4        // GPIO4, activity led, active high
#define TEMP_INC_BTN 7        // GPIO7, increase target temperature, active low
#define TEMP_DEC_BTN 8        // GPIO8, decrease target temperature, active low
#define START_STOP_BTN 9      // GPIO9, start/stop heating, active low
#define HEATER_MOSFET_GATE 38 // GPIO38
#define MAX_ALLOWED_TEMP 55   // °C
#define ADC_RESOLUTION ADC_BITWIDTH_12
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQ 1000 // 1KHz

#define I2C_SDA 13           // GPIO13
#define I2C_SCL 12           // GPIO12
#define I2C_FREQUENCY 100000 // 100Kbit
#define I2C_USED_PORT I2C_NUM_0

float heater_current_I = 0;
float heater_voltage_low_side = 0;
int8_t heater_current_temp_F = 0;
float heater_current_temp_C = 0;
int8_t heater_target_temp_C = 20;
bool is_heating_active = false;
int8_t room_current_temp_F = 0;
float room_current_temp_C = 0;
int8_t mosfet_duty_cycle = 0;

adc_oneshot_unit_handle_t adc2_handle = NULL;
int adcTempVal = 0;
int adcIVal = 0;

static QueueHandle_t gpio_evt_queue = NULL;
static ssd1306_handle_t ssd1306_dev = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void update_oled()
{
    /*ssd1306_write_temp();
    ssd1306_write_symbol();*/
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
    /*
     Current sense circuit is    +12V--2Ohm Heater--22kOhm--10kOhm--GND
     which can be aproximated to +12V---------------22kOhm--10kOhm--GND
     V. divider is used to bring the voltage level after the heater to
     a safe range for the ESP32-S2 ADC.
    */

    adcIVal = 0;
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &adcIVal);

    // adcValue/ADC resolution * Vref * (required resistor/choosen resistor)
    heater_voltage_low_side = adcIVal / (2 ^ ADC_RESOLUTION) * 12;

    // I = V / R
    heater_current_I = heater_voltage_low_side * 2;
}

void read_heater_temp()
{
    // Reference: https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html

    adc_oneshot_read(adc2_handle, ADC_CHANNEL_4, &adcTempVal);
    /*
     NTC thermistor config: +3V3---NTC---10kOhm---GND
     Usual v. divider equation is Vout = Vs * (R0 / ( Rt + R0 ))
     Rearrange to find NTC resistance: Rt = R0 * (( Vs / Vout ) - 1)
     In our case the ADC's Vref is equal to the Vin of the V. divider
     so we can say: adcMax / adcTempVal = Vref / Vout
     With this we can simplify to Rt = R0 * ((adcMax / adcTempVal) - 1)
     ADC on ESP32-S2 is 12-bit, so adcMax is 4095
     */
    heater_current_temp_F = 10000 * ((4095 / adcTempVal) - 1);

    // Fahrenheit --> Celsius
    heater_current_temp_C = (heater_current_temp_F - 32) / 1.8;
}

void read_room_temp()
{
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &adcTempVal);
    room_current_temp_F = 10000 * ((4095 / adcTempVal) - 1);

    // Fahrenheit --> Celsius
    room_current_temp_C = (room_current_temp_F - 32) / 1.8;
}

void isr_change_heater_target_temp(uint8_t value)
{
    if (value)
    {
        mosfet_duty_cycle++;
        if (heater_target_temp_C < MAX_ALLOWED_TEMP)
        {
            heater_target_temp_C++;
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

    if (heater_target_temp_C > 0)
    {
        heater_target_temp_C--;
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
        .bitwidth = ADC_RESOLUTION, // can't use 13 bits, bit 1 always 0, see errata
        .atten = ADC_ATTEN_DB_11,
    };

    // Apply config to Channe0-ADC2, mosfet temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_0, &config));

    // Apply config to Channe3-ADC2, room temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_3, &config));

    // Apply config to Channe4-ADC2, heater temp (middle unit)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config));

    return ESP_OK;
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

esp_err_t init_gpio()
{
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
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, gpio_isr_handler, (void *)1));

    // Decrease temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = TEMP_DEC_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, gpio_isr_handler, (void *)0));

    // Start/Stop temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = START_STOP_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, gpio_isr_handler, NULL));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // Enable per-GPIO interrupts
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

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

    return ESP_OK;
}

void test_oled()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQUENCY;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_USED_PORT, &conf);
    i2c_driver_install(I2C_USED_PORT, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_USED_PORT, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    char data_str[10] = {0};
    sprintf(data_str, "C STR");
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
}

void app_main(void)
{
    init_uart();
    init_gpio();
    init_adc();
    init_pwm();

    char test_str[64];
    static uint8_t led_state = 0;
    int cont = 0;

    while (true)
    {
        read_room_temp();

        // Write data to UART.
        snprintf(test_str, 64, "Test %d, temp is %f °C, current is %f\n",
                 cont,
                 room_current_temp_C,
                 heater_current_I);

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
