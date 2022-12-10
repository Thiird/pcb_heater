#include "inttypes.h"

#define OLED_SAO_BIT 0
#define OLED_ADDRESS 0b011110 || OLED_SAO_BIT << 1
#define SSD1306_CMD 0
#define SSD1306_DATA 1

#define SET_CONTRAST_CONTROL 0x81
#define ENTIRE_DISPLAY_ON 0xA4
#define SET_NORMAL_DISPLAY 0xA6
#define SET_INVERSE_DISPLAY 0xA7
#define SET_OSC_FREQUENCY 0xD5
#define SET_DISPLAY_ON 0xAE
#define SET_DISPLAY_OFF 0xAF
#define SET_MEMORY_ADDRESS_MODE 0x20

uint8_t symbol[] =
    {
        0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
        0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};

void ssd1306_init();
void ssd1306_write_temp();
void ssd1306_write_symbol();