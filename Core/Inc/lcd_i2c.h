#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f1xx_hal.h" // Required for I2C_HandleTypeDef and HAL functions
#include <stdint.h>        // Required for uint8_t

// Define LCD commands and flags
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

// flags for backlight control
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00

#define LCD_EN                  0x04  // Enable bit
#define LCD_RW                  0x02  // Read/Write bit
#define LCD_RS                  0x01  // Register select bit

// Function Prototypes
void LCD_I2C_Init(I2C_HandleTypeDef *hi2c_handle, uint8_t address);
void LCD_I2C_Clear(void);
void LCD_I2C_Home(void);
void LCD_I2C_SetCursor(uint8_t col, uint8_t row);
void LCD_I2C_NoDisplay(void);
void LCD_I2C_Display(void);
void LCD_I2C_NoCursor(void);
void LCD_I2C_Cursor(void);
void LCD_I2C_NoBlink(void);
void LCD_I2C_Blink(void);
void LCD_I2C_ScrollDisplayLeft(void);
void LCD_I2C_ScrollDisplayRight(void);
void LCD_I2C_LeftToRight(void);
void LCD_I2C_RightToLeft(void);
void LCD_I2C_Autoscroll(void);
void LCD_I2C_NoAutoscroll(void);
void LCD_I2C_CreateChar(uint8_t location, uint8_t charmap[]);
void LCD_I2C_Backlight(void);
void LCD_I2C_NoBacklight(void);
void LCD_I2C_PrintChar(char c);
void LCD_I2C_PrintString(const char *str);

#endif // LCD_I2C_H
