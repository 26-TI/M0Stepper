/**
 * @file bsp_can.hpp
 * @author Rh
 * @brief CAN 通信 BSP 层接口
 * @version 1.0
 * @date 2026-07-20
 *
 * @note 基于 TI MSPM0 MCAN 驱动库封装。
 *       标准帧 11-bit ID (0~2047)。
 *       TI MCAN 硬件特性：ID 存储在 Word0 bits[28:18]，内部自动处理。
 *
 * 使用步骤:
 *   1. SysConfig 中配置 MCAN0（引脚、波特率、回环模式选 None）
 *   2. CAN_Init() 初始化
 *   3. CAN_Send() 发送 / CAN_RecvPoll() + CAN_RecvRead() 接收
 *
 * @copyright Copyright (c) 2026
 */

#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * 类型定义
 * =================================================================== */

/** CAN 工作模式 */
typedef enum {
    CAN_MODE_NORMAL   = 0,  /* 真实总线通信 */
    CAN_MODE_LOOPBACK = 1,  /* 内部回环测试 */
} CAN_Mode;

/** CAN 状态信息 */
typedef struct {
    uint32_t txErrCnt;    /* 发送错误计数 */
    uint32_t rxErrCnt;    /* 接收错误计数 */
    uint32_t activity;    /* 0=同步中 1=空闲 2=接收 3=发送 */
    uint32_t lastErrCode; /* 最后错误码 */
    bool     busOff;      /* 总线关闭 */
} CAN_Status;

/* ===================================================================
 * API
 * =================================================================== */

/**
 * @brief 初始化 CAN 模块
 * @param mode  CAN_MODE_NORMAL 或 CAN_MODE_LOOPBACK
 */
void CAN_Init(CAN_Mode mode);

/**
 * @brief 发送标准帧 (11-bit ID)
 * @param id   0~2047
 * @param data 数据指针
 * @param len  数据长度 0~8
 */
bool CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief 轮询接收 (取出 FIFO 消息放入内部缓冲)
 */
void CAN_RecvPoll(void);

/**
 * @brief 从缓冲读出一条消息 (非阻塞)
 * @param id   输出 CAN ID
 * @param data 输出数据
 * @param len  输出长度
 * @return true=有数据, false=无数据
 */
bool CAN_RecvRead(uint32_t *id, uint8_t *data, uint8_t *len);

/**
 * @brief 获取 CAN 状态
 */
void CAN_GetStatus(CAN_Status *status);

/**
 * @brief 切换回环模式 (运行时切换)
 */
void CAN_SetLoopback(bool enable);
void CAN_EnableIrq(void);
void CAN_DisableIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_HPP__ */
