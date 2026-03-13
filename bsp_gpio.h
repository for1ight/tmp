/**
 ******************************************************************************
 * @file    bsp_gpio.h
 * @brief   GPIO驱动 - 继电器控制头文件
 * @note    支持6个球阀继电器控制，高电平有效
 ******************************************************************************
 */

#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* ======================== 继电器定义 ======================== */

/**
 * @brief 继电器编号枚举
 */
typedef enum {
    RELAY_1 = 0,    // 球阀1 (PE7)
    RELAY_2 = 1,    // 球阀2 (PE8)
    RELAY_3 = 2,    // 球阀3 (PE9)
    RELAY_4 = 3,    // 球阀4 (PE10)
    RELAY_5 = 4,    // 球阀5 (PE11)
    RELAY_6 = 5,    // 球阀6 (PE12)
    RELAY_COUNT = 6 // 继电器总数
} BSP_Relay_t;

/* ======================== 中位设置宏 ======================== */

/**
 * @brief 舵机中位脉宽 (微秒)
 * @note  标准舵机: 1500us = 90° (中位)
 */
#define SERVO_MIDPOINT_US    1500

/**
 * @brief 电机ESC中位脉宽 (微秒)
 * @note  ESC标准: 1500us = 停止 (中位)
 */
#define ESC_MIDPOINT_US      1500

/**
 * @brief 舵机中位角度 (度)
 * @note  标准范围: 0-180°, 中位 = 90°
 */
#define SERVO_MIDPOINT_DEG   90

/* ======================== 公共函数 ======================== */

/**
 * @brief 初始化GPIO驱动
 * @note  应在main.c中调用MX_GPIO_Init()之后调用
 * @return true 初始化成功，false 初始化失败
 */
bool BSP_GPIO_Init(void);

/**
 * @brief 设置继电器状态
 * @param[in] relay 继电器编号
 * @param[in] state true=开启(高电平), false=关闭(低电平)
 * @return true 设置成功，false 参数错误
 */
bool BSP_GPIO_SetRelay(BSP_Relay_t relay, bool state);

/**
 * @brief 获取继电器状态
 * @param[in] relay 继电器编号
 * @return true=开启, false=关闭或参数错误
 */
bool BSP_GPIO_GetRelay(BSP_Relay_t relay);

/**
 * @brief 切换继电器状态（反转）
 * @param[in] relay 继电器编号
 * @return 切换后的状态
 */
bool BSP_GPIO_ToggleRelay(BSP_Relay_t relay);

/**
 * @brief 同时设置多个继电器
 * @param[in] relay_mask 继电器掩码 (bit0=relay1, bit1=relay2, ...)
 * @param[in] state 目标状态
 */
void BSP_GPIO_SetRelayMask(uint8_t relay_mask, bool state);

/**
 * @brief 获取所有继电器状态
 * @return 继电器状态掩码 (bit0=relay1状态, ...)
 */
uint8_t BSP_GPIO_GetAllRelayStates(void);

/**
 * @brief 关闭所有继电器 (安全函数)
 */
void BSP_GPIO_DisableAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
