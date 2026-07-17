/**
 * @file bsp_imu601.cpp
 * @brief 汇电籽-601 (ICM42688) UART2 通信层实现（Bsp 层）
 *
 * === 架构 ===
 *
 *   外部数据（UART2 RX）
 *       │
 *       ├─→ ISR（UART2_IRQHandler）     ← 高数据率时触发（FIFO ≥ 4 字节）
 *       │     └─→ ring buffer → imu601_feed_byte()
 *       │
 *       └─→ Polling（bsp_imu601_poll）  ← 每 1ms 兜底，直接读 FIFO
 *             └─→ imu601_feed_byte()
 *                      │
 *                      ▼
 *              imu601 解析器（Device 层）
 *                      │
 *                      ▼
 *              最新 Yaw/Pitch/Roll
 *
 * === 关键设计决策 ===
 *
 *   1. 【为什么不用纯 ISR】
 *      IMP601 数据率 ~240 bytes/200ms，FIFO 阈值 4 字节。
 *      主循环 1ms 一次，每次捞 0~1 字节 → FIFO 永远攒不够 4 字节
 *      → ISR 几乎不触发 → rx_i=0。所以必须加 polling 兜底。
 *
 *   2. 【为什么不发校准指令】
 *      参考例程的校准指令里有一个 float 参数（360.3°），与当前模块
 *      版本不兼容——发送后模块会死机（LED 常亮，不再发数据）。
 *      只发软复位指令是安全的。
 *
 *   3. 【为什么要屏蔽错误中断】
 *      RX 引脚浮空或模块未连接时，UART 会收到噪声 → 帧错误/校验
 *      错误中断泛滥 → CPU 被淹死 → SPI/I2C 全部卡住（Day 5 教训）。
 *
 *   4. 【为什么 ISR 循环加 limit】
 *      防止 FIFO 状态寄存器异常时 ISR 死循环。32 次远大于 8 字节
 *      FIFO 深度，正常情况 1~2 次就排空。
 *
 * === 软复位指令 ===
 *
 *   {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72}
 *   帧头(2) + 设备ID(1) + 指令码 0x12(1) + 数据长度 0x00(1) + 校验和(1)
 *   校验和 = 0x60 + 0x12 + 0x00 = 0x72
 */

#include "bsp_imu601.h"
#include "imu601.h"
#include "ti_msp_dl_config.h"
#include "bsp_uart.h"

/* ================================================================
 *  Ring Buffer（ISR → 主循环 异步传递字节）
 *
 *  128 字节，2 的幂，用 & 0x7F 代替 % 128（M0+ 无硬件除法）
 *  head（ISR 写）/ tail（主循环读），各自独占，无需关中断
 * ================================================================ */
#define RX_BUF_SIZE 128
static volatile uint8_t g_imu_rx_buf[RX_BUF_SIZE];
static volatile uint8_t g_imu_rx_head = 0;
static volatile uint8_t g_imu_rx_tail = 0;
static volatile uint32_t g_imu_rx_total = 0;

static float g_imu_yaw   = 0.0f;
static float g_imu_pitch = 0.0f;
static float g_imu_roll  = 0.0f;

/* ---- ring buffer: 压入一个字节，满返回 0 ---- */
static inline int imu_rx_push(uint8_t b)
{
    uint8_t n = (g_imu_rx_head + 1) & 0x7F;  /* & 0x7F = % 128 */
    if (n == g_imu_rx_tail) return 0;         /* 满，丢弃 */
    g_imu_rx_buf[g_imu_rx_head] = b;
    g_imu_rx_head = n;
    return 1;
}

/* ---- ring buffer: 弹出一个字节，空返回 0 ---- */
static inline int imu_rx_pop(uint8_t *out)
{
    if (g_imu_rx_head == g_imu_rx_tail) return 0;
    *out = g_imu_rx_buf[g_imu_rx_tail];
    g_imu_rx_tail = (g_imu_rx_tail + 1) & 0x7F;
    return 1;
}

/* ================================================================
 *  TX：阻塞式逐字节发送（仅用于短指令，不用 interrupt TX）
 * ================================================================ */
static void imu601_send(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        DL_UART_transmitData(UART_IMU601_INST, data[i]);
        while (DL_UART_isBusy(UART_IMU601_INST)) {}  /* 等硬件发完 */
    }
}

/* ---- 软复位指令（AA 55 60 12 00  crc=60+12+00=72） ---- */
static void imu601_send_reset(void)
{
    uint8_t reset[] = {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};
    imu601_send(reset, sizeof(reset));
    bsp_uart_printf("[IMU601] Reset\r\n");
}

/* ================================================================
 *  ISR — UART2（Extend 类型）
 *
 *  触发条件：RX FIFO ≥ 4 字节（由 SysConfig 阈值决定）
 *  行为：排空 FIFO（最多 32 次）→ 压入 ring buffer
 *  注意：UART2 是 Extend 类型，但 DL_UART_* 基函数对所有类型通用
 * ================================================================ */
extern "C" void UART2_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_IMU601_INST))
    {
        case DL_UART_IIDX_RX:   /* RX 中断（值 = 0x0B） */
        {
            int limit = 32;     /* 硬上限，防止硬件异常时死循环 */
            while (limit-- > 0 && !DL_UART_isRXFIFOEmpty(UART_IMU601_INST))
            {
                uint8_t byte = (uint8_t)DL_UART_receiveData(UART_IMU601_INST);
                g_imu_rx_total++;
                imu_rx_push(byte);
            }
            break;
        }
        default:   /* 错误/超时等中断，读完 IIDX 即清标志，不做额外处理 */
            break;
    }
}

/* ================================================================
 *  公开 API
 * ================================================================ */

/* 手动软复位 */
void bsp_imu601_reset(void)
{
    imu601_send_reset();
}

/* 上电初始化 */
void bsp_imu601_init(void)
{
    imu601_init();              /* 解析器复位 */

    imu601_send_reset();        /* 软复位模块（只复位，不校准！） */

    /* 屏蔽四个错误中断 —— 防止 RX 引脚浮空时中断风暴 */
    DL_UART_disableInterrupt(UART_IMU601_INST,
        DL_UART_INTERRUPT_FRAMING_ERROR |    /* 帧错误 */
        DL_UART_INTERRUPT_PARITY_ERROR  |    /* 校验错误 */
        DL_UART_INTERRUPT_BREAK_ERROR   |    /* 断线错误 */
        DL_UART_INTERRUPT_OVERRUN_ERROR);    /* FIFO 溢出 */

    /* 排空软复位期间可能进入 FIFO 的残留字节 */
    while (!DL_UART_isRXFIFOEmpty(UART_IMU601_INST))
        DL_UART_receiveData(UART_IMU601_INST);

    /* 清 NVIC 残留标志 + 使能 UART2 中断 */
    NVIC_ClearPendingIRQ(UART_IMU601_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IMU601_INST_INT_IRQN);
}

/**
 * 主循环每 1ms 调用一次。
 *
 * 两条收数据路径：
 *   1. 消费 ISR 写入 ring buffer 的字节
 *   2. 【兜底】直接轮询 RX FIFO——ISR 没触发也能收到
 *
 * 注意：polling 路径不会和 ISR 冲突。ISR 先于 main loop 运行
 * （CPU 响应中断时 main loop 被暂停），不会同时读同一个 FIFO。
 */
void bsp_imu601_poll(void)
{
    uint8_t byte;

    /* 路径 1：ring buffer（ISR 写入的） */
    while (imu_rx_pop(&byte))
        imu601_feed_byte(byte);

    /* 路径 2：直接轮询 RX FIFO（ISR 没触发时的兜底） */
    int limit = 32;
    while (limit-- > 0 && !DL_UART_isRXFIFOEmpty(UART_IMU601_INST))
    {
        byte = (uint8_t)DL_UART_receiveData(UART_IMU601_INST);
        g_imu_rx_total++;
        imu601_feed_byte(byte);
    }

    /* 有新解析结果 → 更新缓存 */
    if (imu601_has_new_data())
    {
        IMU601_Data d = imu601_get_data();
        g_imu_yaw   = d.yaw;
        g_imu_pitch = d.pitch;
        g_imu_roll  = d.roll;
    }
}

float    bsp_imu601_get_yaw(void)   { return g_imu_yaw; }
float    bsp_imu601_get_pitch(void) { return g_imu_pitch; }
float    bsp_imu601_get_roll(void)  { return g_imu_roll; }
uint32_t bsp_imu601_rx_total(void)  { return g_imu_rx_total; }
