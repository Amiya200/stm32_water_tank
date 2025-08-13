#include "lcd_i2c.h"
#include "main.h" // For HAL_Delay

// Private variables
static I2C_HandleTypeDef *i2c_handle_g;
static uint8_t lcd_address_g;
static uint8_t _displayfunction;
static uint8_t _displaycontrol;
static uint8_t _displaymode;
static uint8_t _numlines;
static uint8_t _cols;
static uint8_t _backlightval;

// Private function prototypes
static void lcd_send(uint8_t value, uint8_t mode);
static void lcd_write4bits(uint8_t value);
static void lcd_expanderWrite(uint8_t _data);
static void lcd_pulseEnable(uint8_t _data);

/**
  * @brief  Initializes the LCD 16x2 via I2C.
  * @param  hi2c_handle: Pointer to the I2C handle (e.g., &hi2c2 from main.c)
  * @param  address: I2C address of the LCD module (e.g., 0x27 << 1 or 0x3F << 1)
  * @retval None
  */
void LCD_I2C_Init(I2C_HandleTypeDef *hi2c_handle, uint8_t address)
{
    i2c_handle_g = hi2c_handle;
    lcd_address_g = address;

    _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    _backlightval = LCD_BACKLIGHT;
    _numlines = 2;
    _cols = 16;

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // for i2c, we should allow some time after the device is powered up
    HAL_Delay(50);

    // put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0x03 << 4);
    HAL_Delay(5); // wait min 4.1ms

    // second try
    lcd_write4bits(0x03 << 4);
    HAL_Delay(5); // wait min 4.1ms

    // third go!
    lcd_write4bits(0x03 << 4);
    HAL_Delay(1);

    // finally, set to 4-bit interface
    lcd_write4bits(0x02 << 4);

    // set # lines, font size, etc.
    lcd_send(LCD_FUNCTIONSET | _displayfunction, 0);

    // turn the display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    LCD_I2C_Display();

    // clear it off
    LCD_I2C_Clear();

    // set the entry mode
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd_send(LCD_ENTRYMODESET | _displaymode, 0);

    // go home
    LCD_I2C_Home();
}

/**
  * @brief  Clears the LCD display and sets cursor to home position.
  * @param  None
  * @retval None
  */
void LCD_I2C_Clear(void)
{
    lcd_send(LCD_CLEARDISPLAY, 0); // clear display, set cursor position to zero
    HAL_Delay(2);                  // this command takes a long time!
}

/**
  * @brief  Sets the cursor to the home position (0,0).
  * @param  None
  * @retval None
  */
void LCD_I2C_Home(void)
{
    lcd_send(LCD_RETURNHOME, 0); // set cursor position to zero
    HAL_Delay(2);                // this command takes a long time!
}

/**
  * @brief  Sets the cursor to the specified column and row.
  * @param  col: Column position (0-15 for 16x2 LCD)
  * @param  row: Row position (0 or 1 for 16x2 LCD)
  * @retval None
  */
void LCD_I2C_SetCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > _numlines) {
        row = _numlines - 1; // we count rows starting w/0
    }
    lcd_send(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
}

/**
  * @brief  Turns off the LCD display.
  * @param  None
  * @retval None
  */
void LCD_I2C_NoDisplay(void)
{
    _displaycontrol &= ~LCD_DISPLAYON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Turns on the LCD display.
  * @param  None
  * @retval None
  */
void LCD_I2C_Display(void)
{
    _displaycontrol |= LCD_DISPLAYON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Turns off the LCD cursor.
  * @param  None
  * @retval None
  */
void LCD_I2C_NoCursor(void)
{
    _displaycontrol &= ~LCD_CURSORON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Turns on the LCD cursor.
  * @param  None
  * @retval None
  */
void LCD_I2C_Cursor(void)
{
    _displaycontrol |= LCD_CURSORON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Turns off the blinking LCD cursor.
  * @param  None
  * @retval None
  */
void LCD_I2C_NoBlink(void)
{
    _displaycontrol &= ~LCD_BLINKON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Turns on the blinking LCD cursor.
  * @param  None
  * @retval None
  */
void LCD_I2C_Blink(void)
{
    _displaycontrol |= LCD_BLINKON;
    lcd_send(LCD_DISPLAYCONTROL | _displaycontrol, 0);
}

/**
  * @brief  Scrolls the display left without changing the RAM contents.
  * @param  None
  * @retval None
  */
void LCD_I2C_ScrollDisplayLeft(void)
{
    lcd_send(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT, 0);
}

/**
  * @brief  Scrolls the display right without changing the RAM contents.
  * @param  None
  * @retval None
  */
void LCD_I2C_ScrollDisplayRight(void)
{
    lcd_send(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT, 0);
}

/**
  * @brief  Sets the text direction to left-to-right.
  * @param  None
  * @retval None
  */
void LCD_I2C_LeftToRight(void)
{
    _displaymode |= LCD_ENTRYLEFT;
    lcd_send(LCD_ENTRYMODESET | _displaymode, 0);
}

/**
  * @brief  Sets the text direction to right-to-left.
  * @param  None
  * @retval None
  */
void LCD_I2C_RightToLeft(void)
{
    _displaymode &= ~LCD_ENTRYLEFT;
    lcd_send(LCD_ENTRYMODESET | _displaymode, 0);
}

/**
  * @brief  Enables autoscrolling of the display.
  * @param  None
  * @retval None
  */
void LCD_I2C_Autoscroll(void)
{
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_send(LCD_ENTRYMODESET | _displaymode, 0);
}

/**
  * @brief  Disables autoscrolling of the display.
  * @param  None
  * @retval None
  */
void LCD_I2C_NoAutoscroll(void)
{
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_send(LCD_ENTRYMODESET | _displaymode, 0);
}

/**
  * @brief  Creates a custom character for the LCD.
  * @param  location: Character location (0-7)
  * @param  charmap: Array of 8 bytes defining the character pattern
  * @retval None
  */
void LCD_I2C_CreateChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    lcd_send(LCD_SETCGRAMADDR | (location << 3), 0);
    for (int i = 0; i < 8; i++) {
        lcd_send(charmap[i], 1);
    }
}

/**
  * @brief  Turns on the LCD backlight.
  * @param  None
  * @retval None
  */
void LCD_I2C_Backlight(void)
{
    _backlightval = LCD_BACKLIGHT;
    lcd_expanderWrite(0); // Just write 0 to update backlight state
}

/**
  * @brief  Turns off the LCD backlight.
  * @param  None
  * @retval None
  */
void LCD_I2C_NoBacklight(void)
{
    _backlightval = LCD_NOBACKLIGHT;
    lcd_expanderWrite(0); // Just write 0 to update backlight state
}

/**
  * @brief  Prints a single character to the LCD.
  * @param  c: Character to print
  * @retval None
  */
void LCD_I2C_PrintChar(char c)
{
    lcd_send(c, 1);
}

/**
  * @brief  Prints a string to the LCD.
  * @param  str: Pointer to the string to print
  * @retval None
  */
void LCD_I2C_PrintString(const char *str)
{
    while (*str) {
        LCD_I2C_PrintChar(*str++);
    }
}

// Private functions implementations

/**
  * @brief  Sends a command or data byte to the LCD.
  * @param  value: Byte to send
  * @param  mode: 0 for command, 1 for data
  * @retval None
  */
static void lcd_send(uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xF0;
    uint8_t lownib = (value << 4) & 0xF0;
    lcd_write4bits((highnib) | mode);
    lcd_write4bits((lownib) | mode);
}

/**
  * @brief  Writes 4 bits to the LCD.
  * @param  value: 4 bits to write (upper nibble)
  * @retval None
  */
static void lcd_write4bits(uint8_t value)
{
    lcd_expanderWrite(value);
    lcd_pulseEnable(value);
}

/**
  * @brief  Writes data to the I2C expander.
  * @param  _data: Data byte to write
  * @retval None
  */
static void lcd_expanderWrite(uint8_t _data)
{
    uint8_t data_to_send = _data | _backlightval;
    HAL_I2C_Master_Transmit(i2c_handle_g, lcd_address_g, &data_to_send, 1, 100);
}

/**
  * @brief  Pulses the enable pin to latch data.
  * @param  _data: Data byte (including RS, RW, and backlight bits)
  * @retval None
  */
static void lcd_pulseEnable(uint8_t _data)
{
    lcd_expanderWrite(_data | LCD_EN);  // En high
    HAL_Delay(1);                       // enable pulse must be >450ns

    lcd_expanderWrite(_data & ~LCD_EN); // En low
    HAL_Delay(50);                      // commands need > 37us to settle
}
