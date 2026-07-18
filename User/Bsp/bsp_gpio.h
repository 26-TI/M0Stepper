/**
 * @file bsp_gpio.h
 * @brief BSP GPIO 抽象 — 命名引脚控制 + 微秒延时
 *
 * 换 MCU 时对着函数名实现即可：
 *   bsp_cs_high()    → 编码器片选拉高
 *   bsp_dir_high()   → 电机方向 = 正转
 *   bsp_led_toggle() → 调试 LED 翻转
 */

#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_cs_high(void);
void bsp_cs_low(void);
void bsp_dir_high(void);
void bsp_dir_low(void);
void bsp_led_toggle(void);
void bsp_delay_us(uint32_t us);

/* 菜单按键（0=按下/1=松开，SysConfig 上拉） */
int  bsp_btn_up(void);
int  bsp_btn_dn(void);
int  bsp_btn_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H__ */
