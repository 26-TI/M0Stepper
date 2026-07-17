/**
 * @file bsp_gyro.cpp
 * @brief WT901/JY901 陀螺仪 UART1 通信层（Extend 类型，PA18/PA17）
 *
 * 协议：0x5A 帧头，5 字节帧，校验和 = 前 4 字节求和低 8 位
 *   0xAA → Z 轴角速度 wz (°/s)
 *   0xBB → Yaw 角度 (°)
 */

#include "bsp_gyro.h"
#include "gyro.h"
#include "ti_msp_dl_config.h"
#include "bsp_uart.h"

/* ================================================================
 *  Ring Buffer
 * ================================================================ */
#define RX_BUF_SIZE 128
static volatile uint8_t g_rx_buf[RX_BUF_SIZE];
static volatile uint8_t g_rx_head = 0;
static volatile uint8_t g_rx_tail = 0;
static volatile uint32_t g_rx_total = 0;

static float g_latest_wz  = 0.0f;
static float g_latest_yaw = 0.0f;

static inline int rx_buf_push(uint8_t byte)
{
    uint8_t next = (g_rx_head + 1) & 0x7F;
    if (next == g_rx_tail) return 0;
    g_rx_buf[g_rx_head] = byte;
    g_rx_head = next;
    return 1;
}

static inline int rx_buf_pop(uint8_t *out)
{
    if (g_rx_head == g_rx_tail) return 0;
    *out = g_rx_buf[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1) & 0x7F;
    return 1;
}

/* ================================================================
 *  ISR — UART1
 * ================================================================ */
extern "C" void UART1_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_Tly_INST))
    {
        case DL_UART_IIDX_RX:
        {
            int limit = 32;
            while (limit-- > 0 && !DL_UART_isRXFIFOEmpty(UART_Tly_INST))
            {
                uint8_t byte = (uint8_t)DL_UART_receiveData(UART_Tly_INST);
                g_rx_total++;
                rx_buf_push(byte);
            }
            break;
        }
        default:
            break;
    }
}

/* ================================================================
 *  公开函数
 * ================================================================ */

void bsp_gyro_init(void)
{
    gyro_init();

    /* ---- 内部环回自检 ---- */
    bsp_uart_printf("[GYRO] Loopback test on UART1 (PA9/PA8)...\r\n");
    NVIC_DisableIRQ(UART_Tly_INST_INT_IRQN);
    DL_UART_enableLoopbackMode(UART_Tly_INST);

    DL_UART_transmitData(UART_Tly_INST, 0xA5);
    while (DL_UART_isBusy(UART_Tly_INST)) {}

    int ok = 0;
    for (int i = 0; i < 2000; i++)
    {
        if (!DL_UART_isRXFIFOEmpty(UART_Tly_INST))
        {
            uint8_t rx = (uint8_t)DL_UART_receiveData(UART_Tly_INST);
            ok = (rx == 0xA5);
            break;
        }
        for (volatile int d = 0; d < 100; d++) {}
    }

    DL_UART_disableLoopbackMode(UART_Tly_INST);
    bsp_uart_printf("[GYRO] Loopback %s\r\n", ok ? "PASS" : "FAIL");

    /* ---- 屏蔽错误中断 ---- */
    DL_UART_disableInterrupt(UART_Tly_INST,
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR  |
        DL_UART_INTERRUPT_BREAK_ERROR   |
        DL_UART_INTERRUPT_OVERRUN_ERROR);

    /* ---- 清 RX FIFO 残留 ---- */
    while (!DL_UART_isRXFIFOEmpty(UART_Tly_INST))
        DL_UART_receiveData(UART_Tly_INST);

    NVIC_ClearPendingIRQ(UART_Tly_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_Tly_INST_INT_IRQN);
}

void bsp_gyro_poll(void)
{
    uint8_t byte;
    while (rx_buf_pop(&byte))
        gyro_feed_byte(byte);

    if (gyro_has_new_data())
    {
        GyroData d = gyro_get_data();
        g_latest_wz  = d.wz;
        g_latest_yaw = d.yaw;
    }
}

float    bsp_gyro_get_wz(void)   { return g_latest_wz; }
float    bsp_gyro_get_yaw(void)  { return g_latest_yaw; }
uint32_t bsp_gyro_rx_total(void) { return g_rx_total; }
