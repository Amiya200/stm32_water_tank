#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c2;

/* -------- Pin-map helpers --------
   Build a PCF8574 byte from a 4-bit data nibble and RS/EN flags according to LCD_PINMAP.
*/

static inline uint8_t map_nibble_ctrl(uint8_t nibble /*D7..D4 or D4..D7 per map*/,
                                      uint8_t rs, uint8_t en, uint8_t bl_on)
{
#if (LCD_PINMAP == LCD_PINMAP_A)
    /* Map A: D7..D4 -> P7..P4, EN=P2, RW=P1(0), RS=P0, BL=P3
       Byte: [D7 D6 D5 D4 BL EN RW RS] */
    uint8_t b = 0;
    b |= (nibble & 0xF0);           // D7..D4 already in high nibble
    b |= (bl_on ? 0x08 : 0x00);     // BL=P3
    b |= (en ? 0x04 : 0x00);        // EN=P2
    b |= 0x00;                      // RW=P1 forced 0 (write)
    b |= (rs ? 0x01 : 0x00);        // RS=P0
    return b;

#elif (LCD_PINMAP == LCD_PINMAP_B)
    /* Map B: D4..D7 -> P0..P3, EN=P4, RW=P5(0), RS=P6, BL=P7
       Byte: [BL RS RW EN D7 D6 D5 D4] */
    uint8_t b = 0;
    /* nibble provided as high nibble (D7..D4). Shift it down to P3..P0 */
    uint8_t low = (nibble >> 4) & 0x0F;   // D7..D4 -> bits 3..0
    b |= low;                             // D4..D7 on P0..P3
    b |= (en ? 0x10 : 0x00);              // EN=P4
    b |= 0x00;                            // RW=P5 forced 0 (write)
    b |= (rs ? 0x40 : 0x00);              // RS=P6
    b |= (bl_on ? 0x80 : 0x00);           // BL=P7
    return b;
#else
# error "Unsupported LCD_PINMAP selection"
#endif
}

static void expander_write(uint8_t data)
{
    HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, &data, 1, 100);
}

static void pulse_enable(uint8_t data)
{
#if (LCD_PINMAP == LCD_PINMAP_A)
    expander_write(data | 0x04);  // EN=1
    HAL_Delay(1);
    expander_write(data & ~0x04); // EN=0
#elif (LCD_PINMAP == LCD_PINMAP_B)
    expander_write(data | 0x10);  // EN=1
    HAL_Delay(1);
    expander_write(data & ~0x10); // EN=0
#endif
    HAL_Delay(1);
}

static void write4bits(uint8_t nibble /*D7..D4 in high nibble*/, uint8_t rs, uint8_t bl_on)
{
    uint8_t x = map_nibble_ctrl(nibble, rs, 1 /*en edge*/, bl_on);
    expander_write(x);
    pulse_enable(x);
}

/* -------- Public API -------- */

void lcd_backlight_on(void)
{
#if (LCD_PINMAP == LCD_PINMAP_A)
    uint8_t b = 0x08; // BL=1, others 0
#elif (LCD_PINMAP == LCD_PINMAP_B)
    uint8_t b = 0x80; // BL=1, others 0
#endif
    expander_write(b);
}

void lcd_backlight_off(void)
{
    uint8_t b = 0x00;
    expander_write(b);
}

void lcd_send_cmd(uint8_t cmd)
{
    /* high then low nibble, RS=0 */
    write4bits(cmd & 0xF0, 0, 1);
    write4bits((cmd << 4) & 0xF0, 0, 1);
}

void lcd_send_data(uint8_t data)
{
    /* high then low nibble, RS=1 */
    write4bits(data & 0xF0, 1, 1);
    write4bits((data << 4) & 0xF0, 1, 1);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(2);
    lcd_send_cmd(0x80);
}

void lcd_put_cur(uint8_t row, uint8_t col)
{
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_send_cmd(0x80 | (row_offsets[row] + col));
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data((uint8_t)*str++);
}

void lcd_init(void)
{
    HAL_Delay(50);
    lcd_backlight_on();

    /* Force 8-bit mode (send only high-nibble pattern) */
    write4bits(0x30, 0, 1); HAL_Delay(5);
    write4bits(0x30, 0, 1); HAL_Delay(1);
    write4bits(0x30, 0, 1); HAL_Delay(1);

    /* Switch to 4-bit mode */
    write4bits(0x20, 0, 1); HAL_Delay(1);

    /* Function set: 4-bit, 2 lines, 5x8 */
    lcd_send_cmd(0x28); HAL_Delay(1);
    /* Display off */
    lcd_send_cmd(0x08); HAL_Delay(1);
    /* Clear */
    lcd_clear();        HAL_Delay(2);
    /* Entry mode: increment, no shift */
    lcd_send_cmd(0x06); HAL_Delay(1);
    /* Display on, cursor off, blink off */
    lcd_send_cmd(0x0C); HAL_Delay(1);
}

void lcd_self_test(void)
{
    lcd_backlight_on();
    HAL_Delay(150);
    lcd_backlight_off();
    HAL_Delay(150);
    lcd_backlight_on();

    lcd_init();
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("LCD FOUND");
    lcd_put_cur(1, 0);
    lcd_send_string("I2C OK");
}
