#include "lcd_i2c.h"

/** Put this in the src folder **/

extern I2C_HandleTypeDef hi2c2;  // use your I2C handler

void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);

    // Always keep backlight ON
    data_t[0] = data_u | 0x0C | LCD_BACKLIGHT;  // en=1, rs=0
    data_t[1] = data_u | 0x08 | LCD_BACKLIGHT;  // en=0, rs=0
    data_t[2] = data_l | 0x0C | LCD_BACKLIGHT;  // en=1, rs=0
    data_t[3] = data_l | 0x08 | LCD_BACKLIGHT;  // en=0, rs=0

    HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);

    // Always keep backlight ON
    data_t[0] = data_u | 0x0D | LCD_BACKLIGHT;  // en=1, rs=1
    data_t[1] = data_u | 0x09 | LCD_BACKLIGHT;  // en=0, rs=1
    data_t[2] = data_l | 0x0D | LCD_BACKLIGHT;  // en=1, rs=1
    data_t[3] = data_l | 0x09 | LCD_BACKLIGHT;  // en=0, rs=1

    HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(2);       // Wait for clear command to execute
    lcd_send_cmd(0x80); // Set cursor to 0,0
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80; // Line 1
            break;
        case 1:
            col |= 0xC0; // Line 2
            break;
    }
    lcd_send_cmd(col);
}

void lcd_init(void)
{
    HAL_Delay(50);  // wait >40ms after power-on

    // 4-bit init sequence
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20);  // 4-bit mode
    HAL_Delay(10);

    // Function set: 2-line, 5x8 dots
    lcd_send_cmd(0x28);
    HAL_Delay(1);

    // Display OFF
    lcd_send_cmd(0x08);
    HAL_Delay(1);

    // Clear display
    lcd_send_cmd(0x01);
    HAL_Delay(2);

    // Entry mode set: increment cursor
    lcd_send_cmd(0x06);
    HAL_Delay(1);

    // Display ON, Cursor OFF, Blink OFF
    lcd_send_cmd(0x0C);
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}
