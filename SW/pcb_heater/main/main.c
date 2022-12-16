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
#include "math.h"

#define ACTIVITY_LED GPIO_NUM_4        // GPIO4, activity led, active high
#define TEMP_INC_BTN GPIO_NUM_7        // GPIO7, increase target temperature, active low
#define TEMP_DEC_BTN GPIO_NUM_8        // GPIO8, decrease target temperature, active low
#define START_STOP_BTN GPIO_NUM_9      // GPIO9, start/stop heating, active low
#define HEATER_MOSFET_GATE GPIO_NUM_38 // GPIO38
#define MAX_ALLOWED_TEMP 55            // °C
#define ADC_RESOLUTION ADC_BITWIDTH_13
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_DUTY_RES_ACTUAL 8192 // 2 ^ 13
#define LEDC_FREQ 1000            // 1KHz

#define I2C_SDA 13           // GPIO13
#define I2C_SCL 12           // GPIO12
#define I2C_FREQUENCY 100000 // 100 Kb/s
#define I2C_USED_PORT I2C_NUM_0

#define ONE_SECOND 1000 / portTICK_PERIOD_MS
#define DEBOUNCE_TIME_MS 200 / portTICK_PERIOD_MS // 0.2 s debounce time

#define HEATER_RESISTANCE 3.4 // @ 20°C

float heater_current_I = 0;
float heater_voltage_low_side = 0;
int8_t heater_current_temp_F = 0;
float heater_current_temp_C = 0;
int8_t heater_target_temp_C = 20;
bool is_heating_active = false;
int8_t room_temp_F = 0;
float room_temp_C = 0;
int8_t mosfet_duty_cycle = 0;

uint32_t temp_inc_btn_ms = 0;
uint32_t temp_dec_btn_ms = 0;
uint32_t start_stop_btn_ms = 0;

adc_oneshot_unit_handle_t adc2_handle = NULL;
int adcTempVal = 0;
int adcIVal = 0;

adc_oneshot_unit_handle_t adc1_handle = NULL;

static QueueHandle_t gpio_evt_queue = NULL;
static ssd1306_handle_t ssd1306 = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void update_duty_cycle()
{
    // duty cycle formula is ((2 ^ bit_res) - 1) * desired_duty_cycle

    // Set new duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (uint32_t)((LEDC_DUTY_RES_ACTUAL - 1) * (mosfet_duty_cycle / 100.0))));
    // Update duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    printf("Duty cycle: %d = %ld\n", mosfet_duty_cycle, (uint32_t)((LEDC_DUTY_RES_ACTUAL - 1) * (mosfet_duty_cycle / 100.0)));
    printf("value in percent is %f\n", (mosfet_duty_cycle / 100.0));
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

void vTaskUserButtonsListener(void *arg)
{
    uint32_t io_num = -1;
    while (true)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (io_num == TEMP_INC_BTN)
            {
                if ((xTaskGetTickCount() - temp_inc_btn_ms) > DEBOUNCE_TIME_MS)
                {
                    printf("Button event: Temperature Increase\n");
                    mosfet_duty_cycle = (mosfet_duty_cycle != 100) ? mosfet_duty_cycle + 5 : 100;
                    update_duty_cycle();
                    temp_inc_btn_ms = xTaskGetTickCount();
                }
            }

            if (io_num == TEMP_DEC_BTN)
            {
                if ((xTaskGetTickCount() - temp_dec_btn_ms) > DEBOUNCE_TIME_MS)
                {
                    printf("Button event: Temperature Decrease\n");
                    mosfet_duty_cycle = mosfet_duty_cycle == 0 ? 0 : mosfet_duty_cycle - 5;
                    update_duty_cycle();
                    temp_dec_btn_ms = xTaskGetTickCount();
                }
            }

            if (io_num == START_STOP_BTN)
            {
                if ((xTaskGetTickCount() - start_stop_btn_ms) > DEBOUNCE_TIME_MS)
                {
                    mosfet_duty_cycle = 0;
                    update_duty_cycle();
                    printf("Duty cycle is now %d\n", mosfet_duty_cycle);
                    printf("Button event: Start/Stop\n");
                    start_stop_btn_ms = xTaskGetTickCount();
                }
            }
        }
    }
}

void vReadHeaterCurrent()
{
    while (true)
    {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_9, &adcIVal);

        float voltage = (adcIVal / 8192.0) * 12;
        float current = voltage / HEATER_RESISTANCE;
        printf("Heater voltage %.2f V, current %.2f A\n", voltage, current);

        vTaskDelay(ONE_SECOND);
    }
}

void vReadHeaterTemp()
{
    while (true)
    {
        vTaskDelay(ONE_SECOND);
    }
}

void vReadRoomTemp()
{
    while (true)
    {
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_0, &adcTempVal);
        room_temp_F = 10000 * ((8191 / adcTempVal) - 1);

        // Fahrenheit --> Celsius
        room_temp_C = (room_temp_F - 32) / 1.8;

        int Vout = adcTempVal * 3.3 / 8191;
        int Rt = 10000 * Vout / (3.3 - Vout);

        int T = 1 / (1 / 298.15 + log(Rt / 10000) / 3950.0); // Temperature in Kelvin
        int Tc = T - 273.15;                                 // Celsius
        printf("room temp is %d\n", Tc);
        vTaskDelay(ONE_SECOND);
    }
}

void vUpdateOled()
{
    ssd1306 = ssd1306_create(I2C_USED_PORT, SSD1306_I2C_ADDRESS);
    ssd1306_clear_screen(ssd1306, 0);
    uint8_t fill = 0;

    while (true)
    {
        ssd1306_fill_rectangle(ssd1306, 0, 0, 100, 20, fill);
        fill = !fill;
        vTaskDelay(ONE_SECOND);
    }
}

esp_err_t init_gpio()
{
    // Activity LED
    gpio_config_t io_conf = {
        io_conf.intr_type = GPIO_INTR_DISABLE,       // disable interrupt
        io_conf.mode = GPIO_MODE_OUTPUT,             // set as output mode
        io_conf.pin_bit_mask = 1ULL << ACTIVITY_LED, // bit mask of the pins that you want to set
        io_conf.pull_down_en = 0,                    // disable pull-down mode
        io_conf.pull_up_en = 0};                     // disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Increase temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << TEMP_INC_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Decrease temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << TEMP_DEC_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Start/Stop temp button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << START_STOP_BTN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // start gpio task
    xTaskCreate(vTaskUserButtonsListener, "gpio_task_example", 2048, NULL, 10, NULL);

    // Enable per-GPIO interrupts
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // Hook ISR handler for each button
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_INC_BTN, gpio_isr_handler, (void *)TEMP_INC_BTN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TEMP_DEC_BTN, gpio_isr_handler, (void *)TEMP_DEC_BTN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(START_STOP_BTN, gpio_isr_handler, (void *)START_STOP_BTN));

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

esp_err_t init_i2c()
{
    // OLED i2c port
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // already on board
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE, // already on board
        .master.clk_speed = I2C_FREQUENCY,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_USED_PORT, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_USED_PORT, I2C_MODE_MASTER, 0, 0, 0));

    return ESP_OK;
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
    adc_oneshot_chan_cfg_t config2 = {
        .bitwidth = ADC_RESOLUTION, // can't use 13 bits, bit 1 always 0, see errata
        .atten = ADC_ATTEN_DB_11,
    };

    // Apply config to Channe0-ADC2, mosfet temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_0, &config2));

    // Apply config to Channe3-ADC2, room temp
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_3, &config2));

    // Apply config to Channe4-ADC2, heater temp (middle unit)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config2));

    // Configure ADC1 instance
    adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1_handle));

    // Create config for ADC channel
    adc_oneshot_chan_cfg_t config1 = {
        .bitwidth = ADC_RESOLUTION, // can't use 13 bits, bit 1 always 0, see errata
        .atten = ADC_ATTEN_DB_11,
    };

    // Apply config to Channe9-ADC1, heater current
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_9, &config1));

    return ESP_OK;
}

void vTaskActivityLed()
{
    while (true)
    {
        gpio_set_level(ACTIVITY_LED, 1);
        vTaskDelay(ONE_SECOND);
        gpio_set_level(ACTIVITY_LED, 0);
        vTaskDelay(ONE_SECOND);
    }
}

void app_main(void)
{
    // init_uart();

    init_gpio();
    init_adc();
    init_pwm();
    init_i2c();

    xTaskCreate(vTaskActivityLed, "blink activity led", 2048, NULL, 5, NULL);
    xTaskCreate(vTaskUserButtonsListener, "user buttons listener", 2048, NULL, 5, NULL);
    xTaskCreate(vUpdateOled, "update oled", 2048, NULL, 5, NULL);
    xTaskCreate(vReadHeaterCurrent, "read heater current", 2048, NULL, 10, NULL);
    xTaskCreate(vReadHeaterTemp, "read heater temp", 2048, NULL, 10, NULL);
    // xTaskCreate(vReadRoomTemp, "read room temp", 2048, NULL, 10, NULL);
}
