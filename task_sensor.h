/**
 ******************************************************************************
 * @file    task_sensor.h
 * @brief   传感器采集任务头文件
 ******************************************************************************
 */

#ifndef __TASK_SENSOR_H
#define __TASK_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* ======================== 公共函数 ======================== */

/**
 * @brief 传感器采集任务入口 (由 FreeRTOS 调用)
 */
void StartSensorTask(void *argument);

/**
 * @brief 获取传感器数据
 */
bool SensorTask_GetData(SensorData_t *data);

/* ======================== 调试接口 ======================== */

/**
 * @brief 获取ADC统计信息 (调试用)
 * @param[out] sample_count 采样计数
 * @param[out] error_count 错误计数
 * @return 返回 true 表示成功
 */
bool SensorTask_GetADCStats(uint32_t *sample_count, uint32_t *error_count);

/**
 * @brief 获取最后一次有效的ADC数据 (调试用)
 * @param[out] pressure_pa 压力值 (Pa)
 * @param[out] depth_m 深度值 (m)
 * @param[out] temperature 温度值 (℃)
 * @return 返回 true 表示数据有效
 */
bool SensorTask_GetLastADCData(float *pressure_pa, float *depth_m, float *temperature);

/**
 * @brief 重置ADC统计计数器 (调试用)
 */
void SensorTask_ResetADCStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_SENSOR_H */
