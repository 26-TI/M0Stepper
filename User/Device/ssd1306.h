/**
 * @file ssd1306.h
 * @brief SSD1306 OLED 128x64 I2C 驱动（器件层）
 *
 * 依赖：bsp_i2c.h
 * 与 MCU 无关，换平台只需 Bsp 层实现同名函数
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
void ssd1306_putc(char c);
void ssd1306_puts(const char *s);
void ssd1306_printf(uint8_t x, uint8_t y, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */
