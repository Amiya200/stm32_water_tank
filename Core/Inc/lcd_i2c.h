#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h" // HAL definitions

// LCD I2C address (change if needed, common values: 0x4E, 0x7E)
#define SLAVE_ADDRESS_LCD 0x4E

// Backlight control
#define LCD_BACKLIGHT    0x08
#define LCD_NOBACKLIGHT  0x00

// Function prototypes
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_init(void);
void lcd_send_string(char *str);

#endif /* LCD_I2C_H */
