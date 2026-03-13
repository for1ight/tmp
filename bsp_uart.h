/**
 ******************************************************************************
 * @file    bsp_uart.h
 * @brief   多路 UART 通信驱动头文件 - 支持普通和单总线模式
 * @note    UART1/2: 已有驱动（通讯 + IMU）
 *          UART3/4: 普通模式 DMA自动接收 (电机反馈)
 *          UART5: 单总线模式主动查询 (舵机)
 ******************************************************************************
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* ======================== 配置参数 ======================== */
#define UART3_RX_BUF_SIZE      256   // UART3 (电机4) DMA接收缓冲
#define UART4_RX_BUF_SIZE      256   // UART4 (电机3) DMA接收缓冲
#define UART5_RX_BUF_SIZE       64   // UART5 (舵机5) 单总线接收缓冲
#define UART5_QUERY_TIMEOUT_MS  50   // 舵机查询超时时间

/* ======================== 电机反馈数据结构 ======================== */

/**
 * @brief 电机反馈数据包 (UART3/4, 自动反馈, 10字节)
 * Byte0: 温度 (℃)
 * Byte1-2: 电压 (0.01V, 高字节在前)
 * Byte3-4: 电流 (0.01A, 高字节在前)
 * Byte5-6: 电量 (mAh, 高字节在前)
 * Byte7-8: 转速 (100RPM, 高字节在前)
 * Byte9: CRC8校验
 */
typedef struct __attribute__((packed)) {
    uint8_t temp;              // Byte0: 温度 (℃)
    uint8_t voltage_h;         // Byte1: 电压高字节
    uint8_t voltage_l;         // Byte2: 电压低字节
    uint8_t current_h;         // Byte3: 电流高字节
    uint8_t current_l;         // Byte4: 电流低字节
    uint8_t capacity_h;        // Byte5: 电量高字节
    uint8_t capacity_l;        // Byte6: 电量低字节
    uint8_t speed_h;           // Byte7: 转速高字节
    uint8_t speed_l;           // Byte8: 转速低字节
    uint8_t crc;               // Byte9: CRC8校验
} MotorFeedback_t;



/* ======================== API 接口 ======================== */

/**
 * @brief 初始化多路 UART
 *   - UART3: DMA接收模式 (电机4自动反馈)
 *   - UART4: DMA接收模式 (电机3自动反馈)
 *   - UART5: 单总线半双工模式 (舵机查询)
 */
void BSP_MultiUART_Init(void);

/* ============ UART3 (电机4) 接收 API ============ */

/**
 * @brief 获取UART3接收缓冲指针
 */
uint8_t* BSP_UART3_GetRxBuffer(void);

/**
 * @brief 获取UART3当前接收字节数
 */
uint16_t BSP_UART3_GetRxCount(void);

/**
 * @brief 清空UART3接收缓冲
 */
void BSP_UART3_ClearRxBuffer(void);

/* ============ UART4 (电机3) 接收 API ============ */

/**
 * @brief 获取UART4接收缓冲指针
 */
uint8_t* BSP_UART4_GetRxBuffer(void);

/**
 * @brief 获取UART4当前接收字节数
 */
uint16_t BSP_UART4_GetRxCount(void);

/**
 * @brief 清空UART4接收缓冲
 */
void BSP_UART4_ClearRxBuffer(void);

/* ============ UART5 (舵机) 单总线 API ============ */

/**
 * @brief 舵机初始化 - 设置ID号 (UART5 单总线)
 * @param current_id 舵机当前ID (0表示未编程的舵机)
 * @param target_id 要设置的目标ID (1-254)
 * @return HAL_OK-成功, HAL_TIMEOUT-超时, HAL_ERROR-通信错误
 */
HAL_StatusTypeDef BSP_Servo_SetID(uint8_t current_id, uint8_t target_id);

/**
 * @brief 舵机位置查询 (UART5 单总线)
 * @param servo_id 舵机ID (1-254)
 * @param angle 返回角度指针 (单位: 度, 范围0-180)
 * @return HAL_OK-成功, HAL_TIMEOUT-超时, HAL_ERROR-通信错误
 */
HAL_StatusTypeDef BSP_Servo_QueryAngle(uint8_t servo_id, float *angle);

/* ============ 校验函数 ============ */

/**
 * @brief 计算CRC8校验 (电机反馈数据包 Byte9)
 * @note  算法: x^8+x^7+x^5+x^4+1 (0xF7多项式)
 */
uint8_t BSP_CalcChecksum(const uint8_t *data, uint16_t len);

/**
 * @brief 计算CRC16校验码 (高级位通讯帧, 标准CCITT)
 * @note  算法: x^16+x^12+x^5+1 (0x1021多项式)
 */
uint16_t BSP_CalcCRC16(const uint8_t *data, uint16_t len);

/* ======================== 全局变量导出 ======================== */

/**
 * @brief UART3接收字节计数器 (电机4)
 */
extern volatile uint16_t g_UART3_RxCount;

/**
 * @brief UART4接收字节计数器 (电机3)
 */
extern volatile uint16_t g_UART4_RxCount;

/**
 * @brief UART5接收完成信号量 (舵机)
 */
extern osSemaphoreId_t g_UART5_RxCompleteSem;

#endif /* __BSP_UART_H */
