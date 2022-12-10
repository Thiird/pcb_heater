#include "i2c.h"

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
        .clk_flags = 0};
    ESP_ERROR_CHECK(i2c_param_config(I2C_USED_PORT, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_USED_PORT, I2C_MODE_MASTER, NULL, NULL, ESP_INTR_FLAG_LEVEL6));

    return ESP_OK;
}

void i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write(cmd, data, len, true); // size_t data_len?
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_USED_PORT, cmd, true);

    i2c_cmd_link_delete(cmd);
}