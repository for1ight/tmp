/**
 ******************************************************************************
 * @file    task_cmd.h
 * @brief   命令接收任务头文件
 ******************************************************************************
 */

#ifndef __TASK_CMD_H
#define __TASK_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* ======================== 常量定义 ======================== */
#define CMD_BUFFER_SIZE  64

/* ======================== 公共函数 ======================== */

/**
 * @brief 命令接收任务入口 (由 FreeRTOS 调用)
 */
void StartCmdTask(void *argument);

/**
 * @brief 获取最新命令
 */
bool CmdTask_GetCommand(Command_t *cmd);

/**
 * @brief 发送命令应答
 */
void CmdTask_SendAck(uint8_t cmd_id, bool success);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_CMD_H */
