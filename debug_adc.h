/**
 ******************************************************************************
 * @file    debug_adc.h
 * @brief   ADC调试接口 - 用于与上位机通信和数据验证
 ******************************************************************************
 */

#ifndef __DEBUG_ADC_H
#define __DEBUG_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======================== 调试命令定义 ======================== */

/**
 * @brief 调试命令类型
 */
typedef enum {
    DEBUG_CMD_GET_ADC_RAW = 0x01,         // 获取原始ADC值
    DEBUG_CMD_GET_ADC_PROCESSED = 0x02,   // 获取处理后的数据
    DEBUG_CMD_GET_STATISTICS = 0x03,      // 获取统计信息
    DEBUG_CMD_RESET_STATISTICS = 0x04,    // 重置统计信息
    DEBUG_CMD_GET_CALIBRATION = 0x05,     // 获取标定参数
    DEBUG_CMD_SET_CALIBRATION = 0x06,     // 设置标定参数
    DEBUG_CMD_ADC_ENABLE = 0x07,          // 启用ADC
    DEBUG_CMD_ADC_DISABLE = 0x08,         // 禁用ADC
    DEBUG_CMD_ADC_STATUS = 0x09           // 查询ADC状态
} DebugADC_CommandType_t;

/* ======================== 数据包定义 ======================== */

/**
 * @brief ADC调试响应数据包 (发送给上位机)
 */
typedef struct __attribute__((packed)) {
    uint8_t header;                    // 0xAA
    uint8_t command;                   // 命令类型
    uint16_t length;                   // 数据长度
    uint8_t data[256];                 // 数据区
    uint8_t checksum;                  // 校验和
    uint8_t tail;                      // 0x55
} DebugADC_ResponsePacket_t;

/**
 * @brief ADC原始数据包 (用于调试显示)
 */
typedef struct __attribute__((packed)) {
    uint16_t pressure_raw;
    uint16_t temp_board_raw;
    uint16_t battery_volt_raw;
    uint16_t temp_chip_raw;
    uint16_t vrefint_raw;
    uint16_t vbat_raw;
    uint32_t timestamp_ms;
} DebugADC_RawDataPacket_t;

/**
 * @brief ADC处理后的数据包 (用于调试显示)
 */
typedef struct __attribute__((packed)) {
    float pressure_pa;
    float pressure_bar;
    float temp_board;
    float battery_voltage;
    float temp_chip;
    float vref_voltage;
    uint32_t timestamp_ms;
} DebugADC_ProcessedDataPacket_t;

/**
 * @brief ADC统计信息包 (用于调试显示)
 */
typedef struct __attribute__((packed)) {
    uint32_t total_samples;
    uint32_t dma_complete_count;
    uint32_t dma_error_count;
    float avg_pressure;
    float min_pressure;
    float max_pressure;
    float avg_battery_volt;
    float min_battery_volt;
    float max_battery_volt;
    uint8_t adc_enabled;
    uint8_t dma_running;
} DebugADC_StatisticsPacket_t;

/**
 * @brief ADC标定参数包 (用于调试显示)
 */
typedef struct __attribute__((packed)) {
    uint16_t pressure_adc_min;
    uint16_t pressure_adc_max;
    float pressure_min;
    float pressure_max;
    uint16_t battery_adc_min;
    uint16_t battery_adc_max;
    float battery_volt_min;
    float battery_volt_max;
} DebugADC_CalibrationPacket_t;

/* ======================== 初始化和处理函数 ======================== */

/**
 * @brief 初始化ADC调试接口
 * @return 返回 true 表示成功
 */
bool DebugADC_Init(void);

/**
 * @brief 处理调试命令 (从上位机接收到的命令)
 * @param[in] command 命令类型
 * @param[in] data 命令数据指针 (可选)
 * @param[in] length 命令数据长度
 * @param[out] response 响应数据缓冲区
 * @param[out] resp_len 响应数据长度
 * @return 返回 true 表示成功
 */
bool DebugADC_ProcessCommand(uint8_t command, const uint8_t *data, uint16_t length, 
                              uint8_t *response, uint16_t *resp_len);

/**
 * @brief 构建响应数据包
 * @param[out] packet 数据包指针
 * @param[in] command 命令类型
 * @param[in] data 数据指针
 * @param[in] length 数据长度
 * @return 返回 true 表示成功
 */
bool DebugADC_BuildResponsePacket(DebugADC_ResponsePacket_t *packet, 
                                   uint8_t command, const uint8_t *data, uint16_t length);

/**
 * @brief 计算数据包校验和
 * @param[in] data 数据指针
 * @param[in] length 数据长度
 * @return 返回校验和值
 */
uint8_t DebugADC_CalculateChecksum(const uint8_t *data, uint16_t length);

/* ======================== 上位机通信接口 ======================== */

/**
 * @brief 发送原始ADC数据到上位机 (周期性)
 * @return 返回 true 表示成功
 */
bool DebugADC_SendRawData(void);

/**
 * @brief 发送处理后的ADC数据到上位机 (周期性)
 * @return 返回 true 表示成功
 */
bool DebugADC_SendProcessedData(void);

/**
 * @brief 发送统计信息到上位机
 * @return 返回 true 表示成功
 */
bool DebugADC_SendStatistics(void);

/**
 * @brief UART接收中断回调 (用于接收调试命令)
 * @param[in] data 接收到的数据
 * @param[in] length 数据长度
 */
void DebugADC_UARTReceiveCallback(const uint8_t *data, uint16_t length);

/* ======================== 实时监测接口 ======================== */

/**
 * @brief 启用实时数据输出 (用于实时监测图表)
 * @param[in] output_type 输出类型 (0=原始值, 1=处理后)
 * @return 返回 true 表示成功
 */
bool DebugADC_EnableRealtimeOutput(uint8_t output_type);

/**
 * @brief 禁用实时数据输出
 */
void DebugADC_DisableRealtimeOutput(void);

/**
 * @brief 检查是否启用了实时输出
 * @return 返回 true 表示启用
 */
bool DebugADC_IsRealtimeOutputEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_ADC_H */
