#ifndef LCD_I2C_H
#define LCD_I2C_H
#include "main.h" // Assuming main.h includes HAL definitions
// Function prototypes for LCD control
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_clear (void);
void lcd_put_cur(int row, int col);
void lcd_init (void);
void lcd_send_string (char *str);
#endif /* __I2C_LCD_H */
