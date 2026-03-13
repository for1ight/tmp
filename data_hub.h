/**
 ******************************************************************************
 * @file    data_hub.h
 * @brief   数据中心 - 实现任务间数据共享
 ******************************************************************************
 */

#ifndef __DATA_HUB_H
#define __DATA_HUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "task_ctrl.h"
#include "task_sensor.h"
#include "task_telemetry.h"
#include "task_cmd.h"
#include <stdbool.h>

/* ======================== 公共函数 ======================== */

/**
 * @brief 初始化数据中心
 */
void DataHub_Init(void);

/* ========== 传感器数据接口 ========== */

/**
 * @brief 写入传感器数据
 */
void DataHub_WriteSensor(const SensorData_t *sensor);

/**
 * @brief 读取传感器数据
 */
bool DataHub_ReadSensor(SensorData_t *sensor);

/* ========== 遥测数据接口 ========== */

/**
 * @brief 写入遥测数据 (执行器反馈)
 */
void DataHub_WriteTelemetry(const TelemetryData_t *telem);

/**
 * @brief 读取遥测数据
 */
bool DataHub_ReadTelemetry(TelemetryData_t *telem);

/* ========== 命令接口 ========== */

/**
 * @brief 写入命令
 */
void DataHub_WriteCommand(const Command_t *cmd);

/**
 * @brief 读取命令
 */
bool DataHub_ReadCommand(Command_t *cmd);

/* ========== 控制输出接口 ========== */

/**
 * @brief 写入控制输出
 */
void DataHub_WriteOutput(const ControlOutput_t *output);

/**
 * @brief 读取控制输出
 */
bool DataHub_ReadOutput(ControlOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_HUB_H */
