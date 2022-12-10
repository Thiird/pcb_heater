#pragma once

#include "driver/i2c.h"
#include "inttypes.h"

#define I2C_SDA 13           // GPIO13
#define I2C_SCL 12           // GPIO12
#define I2C_FREQUENCY 100000 // 100Kbit
#define I2C_USED_PORT I2C_NUM_0
#define I2C_WRITE 0
#define I2C_READ 1

esp_err_t init_i2c();
void i2c_write(uint8_t addr, uint8_t data, uint8_t len);