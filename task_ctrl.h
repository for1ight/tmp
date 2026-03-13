/**
 ******************************************************************************
 * @file    task_ctrl.h
 * @brief   核心控制任务头文件
 ******************************************************************************
 */

#ifndef __TASK_CTRL_H
#define __TASK_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* ======================== 公共函数 ======================== */

/**
 * @brief 控制任务入口 (由 FreeRTOS 调用)
 */
void StartCtrlTask(void *argument);

/**
 * @brief 获取 PID 参数
 */
void CtrlTask_GetPIDParams(uint8_t pid_id, float *kp, float *ki, float *kd);

/**
 * @brief 设置 PID 参数
 */
void CtrlTask_SetPIDParams(uint8_t pid_id, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_CTRL_H */
