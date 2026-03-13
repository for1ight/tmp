/**
 ******************************************************************************
 * @file    task_uplink.h
 * @brief   上行链路任务头文件
 ******************************************************************************
 */

#ifndef __TASK_UPLINK_H
#define __TASK_UPLINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======================== 数据结构 ======================== */

/**
 * @brief 上传帧格式 (上位机协议)
 */
typedef struct __attribute__((packed)) {
    // 帧头
    uint8_t header[2];       // 0xAA 0x55
    uint16_t length;         // 数据长度 (不含帧头帧尾)
    uint8_t frame_type;      // 0x01-状态帧
    uint32_t timestamp;      // 时间戳 (ms)
    
    // 传感器数据
    float roll;              // 横滚角 (度)
    float pitch;             // 俯仰角 (度)
    float yaw;               // 航向角 (度)
    float depth;             // 深度 (米)
    uint16_t pressure_raw;   // 原始压力值
    
    // 执行器反馈 (简化版)
    struct __attribute__((packed)) {
        float angle;
        uint16_t current;
        uint8_t online;
    } servo[3];
    
    struct __attribute__((packed)) {
        uint16_t rpm;
        uint16_t voltage;
        uint16_t current;
        uint8_t temp;
        uint8_t online;
    } esc[2];
    
    // 控制输出
    uint16_t servo_pwm[3];   // 舵机 PWM
    uint16_t esc_pwm[2];     // 电调 PWM
    uint8_t valve_state;     // 电磁阀状态
    uint8_t ctrl_mode;       // 控制模式
    
    // 系统状态
    uint16_t uptime;         // 系统运行时间 (秒)
    uint8_t error_code;      // 错误代码
    
    // 帧尾
    uint16_t checksum;       // CRC 校验
    uint8_t tail;            // 0x0D
    uint8_t footer[2];       // 0x0D 0x0A (备用)
} UplinkFrame_t;

/* ======================== 公共函数 ======================== */

/**
 * @brief 上行链路任务入口 (由 FreeRTOS 调用)
 */
void StartUplinkTask(void *argument);

/**
 * @brief 发送状态帧
 */
bool UplinkTask_SendStatusFrame(const UplinkFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_UPLINK_H */
