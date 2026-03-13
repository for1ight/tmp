/**
 ******************************************************************************
 * @file    bsp_pwm.h
 * @brief   PWM 驱动头文件
 ******************************************************************************
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "main.h"

/**
 * @brief 初始化 PWM 输出
 */
void BSP_PWM_Init(void);

/**
 * @brief 设置舵机 PWM 脉宽
 * @param servo_id 舵机 ID (0-2)
 * @param pulse_us 脉宽 (500-2500 us)
 */
void BSP_PWM_SetServo(uint8_t servo_id, uint16_t pulse_us);

/**
 * @brief 设置电调 PWM 脉宽
 * @param esc_id 电调 ID (0-1)
 * @param pulse_us 脉宽 (1000-2000 us)
 */
void BSP_PWM_SetESC(uint8_t esc_id, uint16_t pulse_us);

/**
 * @brief 停止所有输出 (紧急停止)
 */
void BSP_PWM_StopAll(void);

/**
 * @brief 电调校准模式
 */
void BSP_PWM_ESC_Calibrate(void);

#endif /* __BSP_PWM_H */
