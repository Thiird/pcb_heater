#include "ssd1306.h"
/*
    SSD1306 has the following default row/column structure:
    SEG/COM0--------------------------SEG127|
    |                 LSB                   |
    |                  .                    |
    |                  .                    |
    |                  .                    |
    |                  .                    |
    |                 MSB                   |
    COM63-----------------------------------|
    No row re-mapping and column-remapping will be applied.

    OLED screen used is 128x32 pixels, so we only use
    the driver in range COM0-COM31 SEG0-SEG127.

    I divide it in sections:
    0-------------------- 90-------------127|
    |   TEMPERATURE WORD  |   SYMBOL WORD   |
    10--------------------|-----------------|
    |     TEMPERATURE     |      SYMBOL     |
    31--------------------|-----------------|
    Temperature is a number in °C
    Temperature word tells whether the displayed temp is heater temp, setpoint, room temp
    Symbol is a symbol of current status
    Symbol word tells what the symbol is signaling
*/

void ssd1306_init()
{
    // Set MUX Ratio X
    // Set Display Offset X
    // Set Display Start Line X
    // Set Segment re-map X
    // Set COM Output Scan Direction X

    // Set COM Pins hardware configuration, YES, look examples
    // Set Contrast control, YES, the more contrast the more current drawn
    // Entire Display On, YES
    // Set Normal Display, YES
    // Set Osc Frequency, YES (probably)
    // Enable charge pump regulator, YES (probably)
    // Display On, YES (probably)
}
void ssd1306_write_temp()
{
}

void ssd1306_write_symbol()
{
}