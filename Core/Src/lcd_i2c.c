#include "lcd_i2c.h"
/** Put this in the src folder **/

extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup (e.g., 0x4E or 0x7E)

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0 (Command mode, Enable high)
	data_t[1] = data_u|0x08;  //en=0, rs=0 (Command mode, Enable low)
	data_t[2] = data_l|0x0C;  //en=1, rs=0 (Command mode, Enable high)
	data_t[3] = data_l|0x08;  //en=0, rs=0 (Command mode, Enable low)
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1 (Data mode, Enable high)
	data_t[1] = data_u|0x09;  //en=0, rs=1 (Data mode, Enable low)
	data_t[2] = data_l|0x0D;  //en=1, rs=1 (Data mode, Enable high)
	data_t[3] = data_l|0x09;  //en=0, rs=1 (Data mode, Enable low)
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x01); // Clear display command
	HAL_Delay(2); // Wait for clear command to execute
	lcd_send_cmd (0x80); // Set cursor to home position (0,0)
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80; // Address for first row
            break;
        case 1:
            col |= 0xC0; // Address for second row
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation sequence (standard for PCF8574 based LCDs)
	HAL_Delay(50);  // wait for >40ms after power-on
	lcd_send_cmd (0x30); // Function set (8-bit mode)
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30); // Function set (8-bit mode)
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30); // Function set (8-bit mode)
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // Function set (4-bit mode)
	HAL_Delay(10);

  // Display initialisation commands (after 4-bit mode is set)
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // Clear display
	HAL_Delay(2); // Longer delay for clear display
	lcd_send_cmd (0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); // Display on/off control --> D = 1, C and B = 0. (Display ON, Cursor OFF, Blink OFF)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
