/**
 * @file menu_ui.h
 * @brief 菜单 UI 层（App 层 — 按键输入 + OLED 输出）
 *
 * 依赖：menu_state（Device 层）、ssd1306（Device 层）、bsp_gpio（Bsp 层）
 *
 * 用法：
 *   menu_ui_init(&ui, &state);
 *   menu_init(&state, entries, count);
 *   while (1) {
 *       menu_ui_poll(&ui);     // 扫描按键 + 更新状态
 *       if (menu_ui_in_task(&ui)) {
 *           run_task();        // 任务用自己的 OLED 渲染
 *       } else {
 *           menu_ui_render(&ui);  // 菜单浏览画面
 *       }
 *       vTaskDelay(10);
 *   }
 */

#ifndef __MENU_UI_H__
#define __MENU_UI_H__

#include "menu_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Menu      *state;     /* 指向下层状态机 */
} MenuUI;

/** 绑定状态机 */
void menu_ui_init(MenuUI *ui, Menu *state);

/** 每 10~50ms 调一次：扫描按键 → 更新状态机（不碰 OLED） */
void menu_ui_poll(MenuUI *ui);

/** 渲染菜单画面到 OLED（只在浏览模式调用） */
void menu_ui_render(MenuUI *ui);

/** 返回已确认的菜单项索引，-1=未确认 */
int  menu_ui_selected(MenuUI *ui);

/** 任务运行中返回 1 */
int  menu_ui_in_task(MenuUI *ui);

#ifdef __cplusplus
}
#endif

#endif
