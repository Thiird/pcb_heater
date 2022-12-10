#define OLED_SAO_BIT 0
#define OLED_ADDRESS 0b011110 || OLED_SAO_BIT << 1

void ssd1306_init();
void ssd1306_write_temp();
void ssd1306_write_symbol();