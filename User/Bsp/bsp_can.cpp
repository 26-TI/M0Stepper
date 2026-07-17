/**
 * @file bsp_can.cpp
 * @brief BSP CAN — MSPM0 MCAN0 实现（中断接收 + 查询发送）
 *
 * PA12=CAN_TX, PA13=CAN_RX, 250kbps, CAN 2.0 标准帧
 * RX: FIFO0 中断 → g_can_new_msg 标志
 * TX: Buffer 0 直接写入
 */

#include "bsp_can.h"
#include "ti_msp_dl_config.h"

static volatile bool    g_can_new_msg = false;
static volatile uint8_t g_can_rx_buf[8];
static volatile uint32_t g_can_rx_id;
static volatile uint8_t g_can_rx_len;

void bsp_can_init(void)
{
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);
    g_can_new_msg = false;
}

bool bsp_can_send(uint32_t id, uint8_t *data, uint8_t len)
{
    DL_MCAN_TxBufElement txMsg;

    if (len > 8) len = 8;

    txMsg.id  = id << 18U;
    txMsg.rtr = 0U;
    txMsg.xtd = 0U;
    txMsg.esi = 0U;
    txMsg.dlc = len;
    txMsg.brs = 0U;
    txMsg.fdf = 0U;
    txMsg.efc = 0U;
    txMsg.mm  = 0U;

    for (uint8_t i = 0; i < len; i++)
        txMsg.data[i] = data[i];

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0U, &txMsg);
    DL_MCAN_TXBufAddReq(MCAN0_INST, 0U);
    return true;
}

bool bsp_can_has_msg(void)
{
    return g_can_new_msg;
}

bool bsp_can_recv(uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (!g_can_new_msg) return false;

    __disable_irq();
    *id  = g_can_rx_id;
    *len = g_can_rx_len;
    for (uint8_t i = 0; i < g_can_rx_len && i < 8; i++)
        data[i] = g_can_rx_buf[i];
    g_can_new_msg = false;
    __enable_irq();

    return true;
}

extern "C" void MCAN0_INST_IRQHandler(void)
{
    switch (DL_MCAN_getPendingInterrupt(MCAN0_INST))
    {
        case DL_MCAN_IIDX_LINE1:
        {
            uint32_t stat = DL_MCAN_getIntrStatus(MCAN0_INST);
            DL_MCAN_clearIntrStatus(MCAN0_INST, stat,
                                     DL_MCAN_INTR_SRC_MCAN_LINE_1);

            DL_MCAN_RxFIFOStatus fifoStat;
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStat);

            if (fifoStat.fillLvl > 0)
            {
                DL_MCAN_RxBufElement rxMsg;
                DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0U,
                                    fifoStat.num, &rxMsg);

                g_can_rx_id  = rxMsg.id >> 18U;
                g_can_rx_len = rxMsg.dlc;
                for (uint8_t i = 0; i < rxMsg.dlc && i < 8; i++)
                    g_can_rx_buf[i] = rxMsg.data[i];
                g_can_new_msg = true;

                DL_MCAN_writeRxFIFOAck(MCAN0_INST, fifoStat.num,
                                        fifoStat.getIdx);
            }
            break;
        }
        default:
            break;
    }
}
