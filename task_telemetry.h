/**
 ******************************************************************************
 * @file    task_telemetry.h
 * @brief   执行器反馈任务头文件
 * @note    支持UART3/4电机自动反馈 + UART5舵机单总线查询
 ******************************************************************************
 */

#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"
#include "bsp_uart.h"

/* ======================== 电机数据结构 ======================== */

/**
 * @brief 电机反馈数据 (解析后的结构)
 */
typedef struct {
    // float temp_c;              // 温度 (℃) - 不需要
    // float voltage_v;           // 电压 (V) - 不需要
    // float current_a;           // 电流 (A) - 不需要
    // float capacity_mah;        // 电量 (mAh) - 不需要
    float speed_rpm;           // 转速 (RPM) - 需要
    bool valid;                // 数据有效标志
    uint32_t timestamp;        // 时间戳
} MotorData_t;

/* ======================== 公共函数 ======================== */

/**
 * @brief 遥测任务入口 (由 FreeRTOS 调用)
 * @note  任务功能:
 *        1. 初始化UART3/4/5
 *        2. UART3/4: 处理自动反馈的电机数据
 *        3. UART5: 初始化舵机ID + 定期查询舵机位置
 *        4. 更新DataHub中的遥测数据
 */
void StartTelemetryTask(void *argument);

/**
 * @brief 获取设备在线状态
 */
void TelemetryTask_GetOnlineStatus(bool *servo_online, bool *esc_online);

/**
 * @brief 获取电机数据 (UART3/4解析结果)
 * @param motor_id 电机ID (0=UART3电机4, 1=UART4电机3)
 * @param motor_data 返回的电机数据
 * @return true-数据有效, false-数据无效
 */
bool TelemetryTask_GetMotorData(uint8_t motor_id, MotorData_t *motor_data);

/**
 * @brief 解析单个电机数据包
 * @param data 原始接收数据
 * @param len 数据长度
 * @param motor_data 返回的解析结果
 * @return true-解析成功, false-CRC校验失败或格式错误
 */
bool TelemetryTask_ParseMotorFeedback(const uint8_t *data, uint16_t len, MotorData_t *motor_data);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_TELEMETRY_H */
