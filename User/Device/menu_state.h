/**
 * @file menu_state.h
 * @brief 菜单状态机（Device 层 — 纯逻辑，零硬件依赖）
 *
 * 只管理：光标移动、确认/取消、任务进入/退出。
 * 不碰 OLED、不碰按键、不碰任何硬件。
 */

#ifndef __MENU_STATE_H__
#define __MENU_STATE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *title;
    const char *desc;
} MenuEntry;

typedef struct {
    const MenuEntry *entries;
    int              count;
    int              cursor;
    int              confirmed;   /* OK 被按下的那一帧为 1，上层读取后应清零 */
    int              running;     /* 1=任务运行中 */
} Menu;

/** 初始化 */
void menu_init(Menu *m, const MenuEntry *entries, int count);

/** 上/下/确认 — 每帧调用一次 */
void menu_up(Menu *m);
void menu_down(Menu *m);
void menu_ok(Menu *m);

/** 任务模式切换 */
void menu_enter_task(Menu *m);
void menu_exit_task(Menu *m);
int  menu_in_task(Menu *m);

/** 读取当前状态 */
int  menu_cursor(Menu *m);
int  menu_consume_confirm(Menu *m);   /* 返回 1 并清零，只触发一次 */

#ifdef __cplusplus
}
#endif

#endif
