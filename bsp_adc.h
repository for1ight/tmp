/**
 ******************************************************************************
 * @file    bsp_adc.h
 * @brief   ADC DMA检测和上传驱动程序 - 头文件
 * @note    支持多通道ADC采集、DMA传输、数据处理和调试接口
 ******************************************************************************
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "dma.h"

/* ======================== 宏定义 ======================== */

/** ADC 通道数量 */
#define BSP_ADC_CHANNEL_COUNT    6

/** ADC 采样周期 (ms) - 由外部触发源决定 */
#define BSP_ADC_SAMPLE_PERIOD    10

/** ADC DMA缓冲区大小 (采样数) - 缓冲多个完整转换序列 */
#define BSP_ADC_DMA_BUFFER_SIZE  (BSP_ADC_CHANNEL_COUNT * 10)

/** ADC 分辨率 (12-bit) */
#define BSP_ADC_RESOLUTION       12

/** ADC 满度值 */
#define BSP_ADC_MAX_VALUE        ((1 << BSP_ADC_RESOLUTION) - 1)

/* ======================== ADC通道定义 ======================== */

/**
 * @brief ADC 通道定义
 */
typedef enum {
    BSP_ADC_CH_PRESSURE_SENSOR = 0,  // PA0 - Channel 0 (压力传感器)
    BSP_ADC_CH_TEMP_BOARD,            // PA1 - Channel 1 (电路板温度)
    BSP_ADC_CH_BATTERY_VOLTAGE,       // PA2 - Channel 2 (电池电压)
    BSP_ADC_CH_ONBOARD_TEMP,          // 内部温度传感器
    BSP_ADC_CH_VREFINT,               // 内部参考电压
    BSP_ADC_CH_VBAT                   // 电池电压监测
} BSP_ADC_Channel_t;

/* ======================== 数据结构 ======================== */

/**
 * @brief ADC 转换数据结构
 */
typedef struct {
    uint16_t pressure_raw;      // 原始压力值 (0-4095)
    uint16_t temp_board_raw;    // 原始板温度值 (0-4095)
    uint16_t battery_volt_raw;  // 原始电池电压值 (0-4095)
    uint16_t temp_chip_raw;     // 芯片温度传感器原始值 (0-4095)
    uint16_t vrefint_raw;       // 参考电压原始值 (0-4095)
    uint16_t vbat_raw;          // 电池电压原始值 (0-4095)
} ADC_RawData_t;

/**
 * @brief ADC 处理后的数据结构
 */
typedef struct {
    float pressure_pa;          // 压力 (Pa)
    float pressure_bar;         // 压力 (bar)
    float temp_board;           // 电路板温度 (℃)
    float battery_voltage;      // 电池电压 (V)
    float temp_chip;            // 芯片温度 (℃)
    float vref_voltage;         // 参考电压 (V)
    uint32_t timestamp_ms;      // 采集时间戳 (ms)
    bool valid;                 // 数据有效性标志
} ADC_ProcessedData_t;

/**
 * @brief ADC 统计数据结构 (用于调试)
 */
typedef struct {
    uint32_t total_samples;         // 总采样数
    uint32_t dma_complete_count;    // DMA完成计数
    uint32_t dma_error_count;       // DMA错误计数
    float avg_pressure;             // 平均压力
    float min_pressure;             // 最小压力
    float max_pressure;             // 最大压力
    float avg_battery_volt;         // 平均电池电压
    float min_battery_volt;         // 最小电池电压
    float max_battery_volt;         // 最大电池电压
    bool adc_enabled;               // ADC使能状态
    bool dma_running;               // DMA运行状态
} ADC_Statistics_t;

/* ======================== 初始化和控制函数 ======================== */

/**
 * @brief 初始化ADC和DMA
 * @return 返回 true 表示成功，false 表示失败
 */
bool BSP_ADC_Init(void);

/**
 * @brief 启动ADC DMA采样
 * @return 返回 true 表示成功，false 表示失败
 */
bool BSP_ADC_StartDMA(void);

/**
 * @brief 停止ADC DMA采样
 * @return 返回 true 表示成功，false 表示失败
 */
bool BSP_ADC_StopDMA(void);

/**
 * @brief 检查ADC是否运行
 * @return 返回 true 表示正在运行，false 表示未运行
 */
bool BSP_ADC_IsRunning(void);

/* ======================== 数据读取函数 ======================== */

/**
 * @brief 获取最新的原始ADC数据
 * @param[out] raw_data 原始数据指针
 * @return 返回 true 表示成功，false 表示失败或无新数据
 */
bool BSP_ADC_GetRawData(ADC_RawData_t *raw_data);

/**
 * @brief 获取最新的处理后的ADC数据
 * @param[out] processed_data 处理后数据指针
 * @return 返回 true 表示成功，false 表示失败或无新数据
 */
bool BSP_ADC_GetProcessedData(ADC_ProcessedData_t *processed_data);

/**
 * @brief 获取特定通道的最新转换值
 * @param[in] channel ADC通道
 * @param[out] value 转换值指针
 * @return 返回 true 表示成功，false 表示失败
 */
bool BSP_ADC_GetChannelValue(BSP_ADC_Channel_t channel, uint16_t *value);

/**
 * @brief 获取特定通道的处理后数据
 * @param[in] channel ADC通道
 * @param[out] value_float 处理后值指针 (实际单位: Pa, ℃, V等)
 * @return 返回 true 表示成功，false 表示失败
 */
bool BSP_ADC_GetChannelFloat(BSP_ADC_Channel_t channel, float *value_float);

/* ======================== 数据处理函数 ======================== */

/**
 * @brief 将原始ADC值转换为物理单位
 * @param[in] raw_data 原始数据指针
 * @param[out] processed_data 处理后数据指针
 * @return 返回 true 表示成功
 */
bool BSP_ADC_ProcessData(const ADC_RawData_t *raw_data, ADC_ProcessedData_t *processed_data);

/**
 * @brief 获取原始值平均值 (用于数据滤波)
 * @param[in] channel ADC通道
 * @param[out] avg_value 平均值指针
 * @param[in] sample_count 采样数 (最多 BSP_ADC_DMA_BUFFER_SIZE)
 * @return 返回 true 表示成功
 */
bool BSP_ADC_GetChannelAverage(BSP_ADC_Channel_t channel, uint16_t *avg_value, uint16_t sample_count);

/* ======================== 调试接口 ======================== */

/**
 * @brief 获取ADC统计信息
 * @param[out] stats 统计数据指针
 * @return 返回 true 表示成功
 */
bool BSP_ADC_GetStatistics(ADC_Statistics_t *stats);

/**
 * @brief 重置ADC统计计数器
 * @return 返回 true 表示成功
 */
bool BSP_ADC_ResetStatistics(void);

/**
 * @brief 打印ADC调试信息到UART (需要实现UART接口)
 * @return 返回 true 表示成功
 */
bool BSP_ADC_PrintDebugInfo(void);

/**
 * @brief 设置压力传感器标定参数
 * @param[in] adc_min ADC最小值 (对应最小压力)
 * @param[in] adc_max ADC最大值 (对应最大压力)
 * @param[in] press_min 最小压力 (Pa)
 * @param[in] press_max 最大压力 (Pa)
 * @return 返回 true 表示成功
 */
bool BSP_ADC_SetPressureCalibration(uint16_t adc_min, uint16_t adc_max, float press_min, float press_max);

/**
 * @brief 设置电池电压标定参数
 * @param[in] adc_min ADC最小值 (对应最低电压)
 * @param[in] adc_max ADC最大值 (对应最高电压)
 * @param[in] volt_min 最低电压 (V)
 * @param[in] volt_max 最高电压 (V)
 * @return 返回 true 表示成功
 */
bool BSP_ADC_SetBatteryCalibration(uint16_t adc_min, uint16_t adc_max, float volt_min, float volt_max);

/**
 * @brief 获取当前的标定参数
 * @param[out] pressure_adc_min 压力对应的ADC最小值
 * @param[out] pressure_adc_max 压力对应的ADC最大值
 * @param[out] pressure_min 最小压力 (Pa)
 * @param[out] pressure_max 最大压力 (Pa)
 * @param[out] battery_adc_min 电池对应的ADC最小值
 * @param[out] battery_adc_max 电池对应的ADC最大值
 * @param[out] battery_volt_min 最低电压 (V)
 * @param[out] battery_volt_max 最高电压 (V)
 * @return 返回 true 表示成功
 */
bool BSP_ADC_GetCalibrationParams(
    uint16_t *pressure_adc_min, uint16_t *pressure_adc_max,
    float *pressure_min, float *pressure_max,
    uint16_t *battery_adc_min, uint16_t *battery_adc_max,
    float *battery_volt_min, float *battery_volt_max
);

/* ======================== 中断回调函数 (由DMA调用) ======================== */

/**
 * @brief DMA完成转换回调函数 (由HAL_DMA中断调用)
 * 注意: 此函数在中断上下文中执行，应保持简短
 */
void BSP_ADC_DMA_ConversionComplete(void);

/**
 * @brief DMA半转换回调函数 (用于实时处理)
 * 注意: 此函数在中断上下文中执行，应保持简短
 */
void BSP_ADC_DMA_HalfConversion(void);

/**
 * @brief DMA错误回调函数
 */
void BSP_ADC_DMA_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
