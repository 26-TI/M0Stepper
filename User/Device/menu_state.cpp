/**
 * @file menu_state.cpp
 * @brief 菜单状态机实现 — 纯 C，零 include 依赖（除自身头文件）
 */

#include "menu_state.h"

void menu_init(Menu *m, const MenuEntry *entries, int count)
{
    m->entries   = entries;
    m->count     = count;
    m->cursor    = 0;
    m->confirmed = 0;
    m->running   = 0;
}

void menu_up(Menu *m)
{
    if (m->running) return;
    m->cursor--;
    if (m->cursor < 0) m->cursor = m->count - 1;
}

void menu_down(Menu *m)
{
    if (m->running) return;
    m->cursor++;
    if (m->cursor >= m->count) m->cursor = 0;
}

void menu_ok(Menu *m)
{
    if (m->running)
        menu_exit_task(m);
    else
    {
        m->confirmed = 1;
        m->running   = 1;
    }
}

void menu_enter_task(Menu *m) { m->running = 1; }
void menu_exit_task(Menu *m)  { m->running = 0; m->confirmed = 0; }
int  menu_in_task(Menu *m)    { return m->running; }

int  menu_cursor(Menu *m)           { return m->cursor; }
int  menu_consume_confirm(Menu *m)  { int r = m->confirmed; m->confirmed = 0; return r; }
