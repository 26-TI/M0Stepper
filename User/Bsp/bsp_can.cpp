/**
 * @file bsp_can.cpp
 * @author Rh
 * @brief CAN 通信 BSP 层实现
 * @version 1.0
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 */

#include "bsp_can.hpp"
#include <string.h>

/* ===================================================================
 * 接收环形缓冲
 * =================================================================== */

#define CAN_RX_BUF_SIZE 16

static volatile uint32_t s_rxBufId[CAN_RX_BUF_SIZE];
static volatile uint8_t  s_rxBufData[CAN_RX_BUF_SIZE][8];
static volatile uint8_t  s_rxBufLen[CAN_RX_BUF_SIZE];
static volatile uint8_t  s_rxBufHead = 0;
static volatile uint8_t  s_rxBufTail = 0;

static bool s_loopback = false;

/* ===================================================================
 * 内部辅助
 * =================================================================== */

/**
 * @brief 写受保护寄存器前解锁
 */
static void can_write_unlock(void)
{
    /* CCCR.CCE = 1, CCCR.INIT = 1 */
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR |= (MCAN_CCCR_CCE_MASK | MCAN_CCCR_INIT_MASK);
    while (0U == (MCAN0_INST->MCANSS.MCAN.MCAN_CCCR & MCAN_CCCR_INIT_MASK));
}

/**
 * @brief 写受保护寄存器后锁定
 */
static void can_write_lock(void)
{
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR &= ~MCAN_CCCR_CCE_MASK;
    MCAN0_INST->MCANSS.MCAN.MCAN_CCCR &= ~MCAN_CCCR_INIT_MASK;
    while (0U != (MCAN0_INST->MCANSS.MCAN.MCAN_CCCR & MCAN_CCCR_INIT_MASK));
}

/* ===================================================================
 * API 实现
 * =================================================================== */

void CAN_Init(CAN_Mode mode)
{
    /* SYSCFG_DL_MCAN0_init() 已由 SYSCFG_DL_init() 调用，
       这里设置回环模式和全局滤波器 */

    /* 配置全局滤波器：不匹配的帧也进 FIFO0 */
    DL_MCAN_ConfigParams cfg;
    cfg.monEnable        = 0;
    cfg.asmEnable        = 0;
    cfg.tsSelect         = 0;   /* 不用时间戳 */
    cfg.tsPrescalar      = 0;
    cfg.timeoutSelect    = 0;   /* 不用超时 */
    cfg.timeoutPreload   = 0;
    cfg.timeoutCntEnable = 0;
    cfg.filterConfig.rrfe = 0;  /* 不拒绝远程帧 */
    cfg.filterConfig.rrfs = 0;
    cfg.filterConfig.anfe = 0;  /* 不匹配扩展帧 → FIFO0 */
    cfg.filterConfig.anfs = 0;  /* 不匹配标准帧 → FIFO0 */
    DL_MCAN_config(MCAN0_INST, &cfg);

    s_loopback = (mode == CAN_MODE_LOOPBACK);
    CAN_SetLoopback(s_loopback);
}

bool CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    /* 检查 TX Buffer 0 是否忙 (TXBRP bit0) */
    if (DL_MCAN_getTxBufReqPend(MCAN0_INST) & 0x01)
        return false;   /* 忙，不阻塞，上层决定重试或丢弃 */

    DL_MCAN_TxBufElement txElem;
    memset(&txElem, 0, sizeof(txElem));

    /* TI MCAN: 标准 ID 在 Word0 bits[28:18] */
    txElem.id  = (id & 0x7FF) << 18;
    txElem.xtd = 0;
    txElem.rtr = 0;
    txElem.fdf = 0;
    txElem.efc = 0;
    txElem.dlc = (len > 8) ? 8 : len;
    if (len > 0 && data) memcpy(txElem.data, data, txElem.dlc);

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0, &txElem);
    DL_MCAN_TXBufAddReq(MCAN0_INST, 0);
    return true;
}

void CAN_RecvPoll(void)
{
    DL_MCAN_RxFIFOStatus st;
    st.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);

    while (st.fillLvl > 0)
    {
        DL_MCAN_RxBufElement rxElem;
        DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0,
                           DL_MCAN_RX_FIFO_NUM_0, &rxElem);
        DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0,
                               st.getIdx);

        uint8_t idx = s_rxBufHead;
        s_rxBufId[idx]   = (rxElem.id >> 18) & 0x7FF;
        s_rxBufLen[idx]  = (rxElem.dlc > 8) ? 8 : (uint8_t)rxElem.dlc;
        memcpy((void *)s_rxBufData[idx], rxElem.data, s_rxBufLen[idx]);
        s_rxBufHead = (idx + 1) % CAN_RX_BUF_SIZE;

        st.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);
    }
}

bool CAN_RecvRead(uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (s_rxBufTail == s_rxBufHead) return false;

    uint8_t idx = s_rxBufTail;
    *id  = s_rxBufId[idx];
    *len = s_rxBufLen[idx];
    memcpy(data, (const void *)s_rxBufData[idx], *len);
    s_rxBufTail = (idx + 1) % CAN_RX_BUF_SIZE;
    return true;
}

void CAN_GetStatus(CAN_Status *status)
{
    DL_MCAN_ErrCntStatus ec;
    DL_MCAN_getErrCounters(MCAN0_INST, &ec);

    DL_MCAN_ProtocolStatus ps;
    DL_MCAN_getProtocolStatus(MCAN0_INST, &ps);

    status->txErrCnt    = ec.transErrLogCnt;
    status->rxErrCnt    = ec.recErrCnt;
    status->activity    = ps.act;
    status->lastErrCode = ps.lastErrCode;
    status->busOff      = (ps.busOffStatus != 0);
}

void CAN_SetLoopback(bool enable)
{
    can_write_unlock();

    DL_MCAN_lpbkModeEnable(MCAN0_INST, DL_MCAN_LPBK_MODE_INTERNAL, enable);

    s_loopback = enable;

    can_write_lock();
}

/* ===================================================================
 * 中断模式 (RX 由 ISR 驱动)
 * =================================================================== */

void CAN_EnableIrq(void)
{
    /* 只开 RF0N (FIFO0 新消息) + RF0L (FIFO0 水位) */
    DL_MCAN_enableIntr(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, 0U);
    DL_MCAN_enableIntr(MCAN0_INST,
        DL_MCAN_INTERRUPT_RF0N | DL_MCAN_INTERRUPT_RF0L, 1U);

    /* Line 0 已在 SysConfig 配好 (ILE=1, MSP=1)，这里只清一次 */
    DL_MCAN_clearInterruptStatus(MCAN0_INST, DL_MCAN_MSP_INTERRUPT_LINE0);
    __DSB();
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);
}

void CAN_DisableIrq(void)
{
    NVIC_DisableIRQ(MCAN0_INST_INT_IRQN);
}

/* ---- ISR: 仿 CAN_RecvPoll，读 FIFO 进环形缓冲 ---- */
extern "C" void MCAN0_INST_IRQHandler(void)
{
    uint32_t status = DL_MCAN_getIntrStatus(MCAN0_INST);
    DL_MCAN_clearIntrStatus(MCAN0_INST, status, DL_MCAN_INTR_SRC_MCAN_LINE_0);

    if (status & MCAN_IR_RF0N_MASK)
    {
        /* RF0N 已触发，FIFO 必有数据，直接读 */
        DL_MCAN_RxFIFOStatus st;
        st.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);

        while (st.fillLvl > 0)
        {
            DL_MCAN_RxBufElement rxElem;
            DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0,
                               DL_MCAN_RX_FIFO_NUM_0, &rxElem);
            DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0,
                                   st.getIdx);

            uint8_t idx = s_rxBufHead;
            s_rxBufId[idx]   = (rxElem.id >> 18) & 0x7FF;
            s_rxBufLen[idx]  = (rxElem.dlc > 8) ? 8 : (uint8_t)rxElem.dlc;
            memcpy((void *)s_rxBufData[idx], rxElem.data, s_rxBufLen[idx]);
            s_rxBufHead = (idx + 1) % CAN_RX_BUF_SIZE;

            st.num = DL_MCAN_RX_FIFO_NUM_0;
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &st);
        }
    }
}
