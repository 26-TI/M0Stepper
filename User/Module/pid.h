/**
 * @file pid.h
 * @brief 通用 PID 控制器（纯算法模块，零硬件依赖）
 */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PID 控制器结构体 */
typedef struct
{
    float kp;              ///< 比例系数
    float ki;              ///< 积分系数
    float kd;              ///< 微分系数
    float integral;        ///< 积分累加值
    float prev_error;      ///< 上一次误差（微分用）
    float integral_limit;  ///< 积分限幅（抗饱和）
    float output_limit;    ///< 输出限幅
} PID;

void  pid_init(PID *pid, float kp, float ki, float kd, float ilim, float olim);
float pid_update(PID *pid, float target, float actual, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
