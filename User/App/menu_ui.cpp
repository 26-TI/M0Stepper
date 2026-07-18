/**
 * @file menu_ui.cpp
 * @brief 菜单 UI 实现 — 按键消抖 + OLED 渲染
 *
 * 分拆 poll / render，任务运行时上层跳过 render 用自己的显示逻辑。
 */

#include "menu_ui.h"
#include "ssd1306.h"
#include "bsp_gpio.h"
/* 不直接碰 ti_msp_dl_config.h — 按键走 bsp_gpio，OLED 走 ssd1306 */

/* ---- 消抖：连续 DEBOUNCE 次按下 → 置 latch → 触发一次
         松开后清 latch → 才能再次触发
         按住不放不会重复触发 ---- */
#define DEBOUNCE  3

typedef struct {
    int cnt;
    int latched;   /* 1=已触发，等松开 */
} BtnState;

static BtnState g_up_btn, g_dn_btn, g_ok_btn;

static int read_btn(int (*get)(void), BtnState *s)
{
    if (get())   /* 按下 */
    {
        if (!s->latched && ++s->cnt >= DEBOUNCE)
        {
            s->latched = 1;    /* 锁住，按住期间不再触发 */
            s->cnt     = 0;
            return 1;
        }
    }
    else
    {
        s->cnt     = 0;
        s->latched = 0;        /* 松开 → 解锁 */
    }
    return 0;
}

/* ================================================================
 *  公开 API
 * ================================================================ */

void menu_ui_init(MenuUI *ui, Menu *state)
{
    ui->state = state;
}

void menu_ui_poll(MenuUI *ui)
{
    Menu *m = ui->state;

    /* 扫描按键 → 更新状态机 */
    if (read_btn(bsp_btn_up, &g_up_btn)) menu_up(m);
    if (read_btn(bsp_btn_dn, &g_dn_btn)) menu_down(m);
    if (read_btn(bsp_btn_ok, &g_ok_btn)) menu_ok(m);
}

void menu_ui_render(MenuUI *ui)
{
    Menu *m = ui->state;
    const MenuEntry *e = &m->entries[m->cursor];

    ssd1306_printf(0, 0, "%s", e->title);
    ssd1306_printf(0, 2, "%s", e->desc);

    if (m->running)
        ssd1306_printf(0, 6, "OK: Exit");
    else
        ssd1306_printf(0, 6, "UP/DN:Sel OK:Go");
}

int menu_ui_selected(MenuUI *ui)
{
    return menu_consume_confirm(ui->state) ? menu_cursor(ui->state) : -1;
}

int menu_ui_in_task(MenuUI *ui)
{
    return menu_in_task(ui->state);
}
