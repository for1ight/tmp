/**
 ******************************************************************************
 * @file    bsp_adc.c
 * @brief   ADC DMA检测和上传驱动程序 - 实现文件
 * @note    支持多通道ADC采集、DMA传输、数据处理和调试接口
 ******************************************************************************
 */

#include "bsp_adc.h"
#include "string.h"
#include "cmsis_os.h"

/* ======================== 外部声明 ======================== */

/** DMA句柄 - 由Core/Src/adc.c定义 */
extern DMA_HandleTypeDef hdma_adc1;

/* ======================== 私有宏定义 ======================== */

/** 电压参考 (3.3V) */
#define BSP_ADC_VREF            3.3f

/** 芯片温度传感器参数 (STM32F407)
 *  V25 = 0.76V (25℃时电压)
 *  Avg_Slope = 2.5 mV/℃
 */
#define BSP_ADC_TEMP_V25        0.76f
#define BSP_ADC_TEMP_SLOPE      0.0025f

/** 内部参考电压 (理论值: 1.2V) */
#define BSP_ADC_VREFINT_TYPICAL 1.2f

/* ======================== 私有数据结构 ======================== */

/**
 * @brief ADC驱动内部状态
 */
typedef struct {
    // DMA 缓冲区
    uint16_t dma_buffer[BSP_ADC_DMA_BUFFER_SIZE];
    
    // 当前数据
    ADC_RawData_t current_raw_data;
    ADC_ProcessedData_t current_processed_data;
    
    // 标志位
    bool is_initialized;
    bool is_running;
    bool new_data_available;
    
    // 同步信号量 (在中断中释放)
    osSemaphoreId_t data_ready_semaphore;
    
    // 统计数据
    ADC_Statistics_t statistics;
    
    // 标定参数
    struct {
        uint16_t pressure_adc_min;
        uint16_t pressure_adc_max;
        float pressure_min;
        float pressure_max;
        
        uint16_t battery_adc_min;
        uint16_t battery_adc_max;
        float battery_volt_min;
        float battery_volt_max;
    } calibration;
} ADC_DriverState_t;

/* ======================== 私有全局变量 ======================== */

static ADC_DriverState_t adc_state = {0};

/* ======================== 私有函数声明 ======================== */

/**
 * @brief 初始化DMA缓冲区
 */
static void ADC_DMA_BufferInit(void);

/**
 * @brief 提取一个完整的ADC转换序列数据
 */
static void ADC_ExtractConversionData(uint16_t buffer_offset, ADC_RawData_t *raw_data);

/**
 * @brief 计算通道平均值
 */
static uint16_t ADC_CalculateChannelAverage(uint8_t channel_index, uint16_t sample_count);

/**
 * @brief 原始值到压力值的转换
 */
static float ADC_RawToPressure(uint16_t raw_value);

/**
 * @brief 原始值到电压值的转换
 */
static float ADC_RawToVoltage(uint16_t raw_value);

/**
 * @brief 原始值到温度值的转换
 */
static float ADC_RawToTemperature(uint16_t raw_value);

/* ======================== 初始化和控制函数 ======================== */

bool BSP_ADC_Init(void)
{
    // 如果已初始化，则直接返回
    if (adc_state.is_initialized) {
        return true;
    }
    
    // 初始化DMA缓冲区
    ADC_DMA_BufferInit();
    
    // 初始化标定参数 (默认值)
    adc_state.calibration.pressure_adc_min = 0;
    adc_state.calibration.pressure_adc_max = 4095;
    adc_state.calibration.pressure_min = 0.0f;
    adc_state.calibration.pressure_max = 500000.0f;  // 5 bar
    
    adc_state.calibration.battery_adc_min = 0;
    adc_state.calibration.battery_adc_max = 4095;
    adc_state.calibration.battery_volt_min = 0.0f;
    adc_state.calibration.battery_volt_max = 16.5f;  // 4S LiPo
    
    // 创建数据就绪信号量
    const osSemaphoreAttr_t semaphore_attr = {
        .name = "ADC_DataReady"
    };
    adc_state.data_ready_semaphore = osSemaphoreNew(1, 0, &semaphore_attr);
    
    if (adc_state.data_ready_semaphore == NULL) {
        return false;
    }
    
    // 初始化统计信息
    memset(&adc_state.statistics, 0, sizeof(ADC_Statistics_t));
    adc_state.statistics.min_pressure = 999999.0f;
    adc_state.statistics.max_pressure = 0.0f;
    adc_state.statistics.min_battery_volt = 999.0f;
    adc_state.statistics.max_battery_volt = 0.0f;
    
    // 关联DMA到ADC
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    
    adc_state.is_initialized = true;
    adc_state.is_running = false;
    
    return true;
}

bool BSP_ADC_StartDMA(void)
{
    if (!adc_state.is_initialized) {
        return false;
    }
    
    if (adc_state.is_running) {
        return true;  // 已在运行
    }
    
    // 启动ADC DMA转换
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_state.dma_buffer, BSP_ADC_DMA_BUFFER_SIZE);
    
    if (status != HAL_OK) {
        adc_state.statistics.dma_error_count++;
        return false;
    }
    
    adc_state.is_running = true;
    adc_state.statistics.adc_enabled = true;
    adc_state.statistics.dma_running = true;
    
    return true;
}

bool BSP_ADC_StopDMA(void)
{
    if (!adc_state.is_initialized || !adc_state.is_running) {
        return true;
    }
    
    HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(&hadc1);
    
    if (status != HAL_OK) {
        return false;
    }
    
    adc_state.is_running = false;
    adc_state.statistics.adc_enabled = false;
    adc_state.statistics.dma_running = false;
    
    return true;
}

bool BSP_ADC_IsRunning(void)
{
    return adc_state.is_running;
}

/* ======================== 数据读取函数 ======================== */

bool BSP_ADC_GetRawData(ADC_RawData_t *raw_data)
{
    if (!raw_data || !adc_state.is_initialized) {
        return false;
    }
    
    // 禁用中断，保证数据一致性
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    memcpy(raw_data, &adc_state.current_raw_data, sizeof(ADC_RawData_t));
    bool has_new_data = adc_state.new_data_available;
    adc_state.new_data_available = false;
    
    __set_PRIMASK(primask);
    
    return has_new_data;
}

bool BSP_ADC_GetProcessedData(ADC_ProcessedData_t *processed_data)
{
    if (!processed_data || !adc_state.is_initialized) {
        return false;
    }
    
    // 禁用中断，保证数据一致性
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    memcpy(processed_data, &adc_state.current_processed_data, sizeof(ADC_ProcessedData_t));
    bool has_new_data = adc_state.new_data_available;
    adc_state.new_data_available = false;
    
    __set_PRIMASK(primask);
    
    return has_new_data;
}

bool BSP_ADC_GetChannelValue(BSP_ADC_Channel_t channel, uint16_t *value)
{
    if (!value || channel >= BSP_ADC_CHANNEL_COUNT || !adc_state.is_initialized) {
        return false;
    }
    
    uint16_t raw_value = 0;
    
    switch (channel) {
        case BSP_ADC_CH_PRESSURE_SENSOR:
            raw_value = adc_state.current_raw_data.pressure_raw;
            break;
        case BSP_ADC_CH_TEMP_BOARD:
            raw_value = adc_state.current_raw_data.temp_board_raw;
            break;
        case BSP_ADC_CH_BATTERY_VOLTAGE:
            raw_value = adc_state.current_raw_data.battery_volt_raw;
            break;
        case BSP_ADC_CH_ONBOARD_TEMP:
            raw_value = adc_state.current_raw_data.temp_chip_raw;
            break;
        case BSP_ADC_CH_VREFINT:
            raw_value = adc_state.current_raw_data.vrefint_raw;
            break;
        case BSP_ADC_CH_VBAT:
            raw_value = adc_state.current_raw_data.vbat_raw;
            break;
        default:
            return false;
    }
    
    *value = raw_value;
    return true;
}

bool BSP_ADC_GetChannelFloat(BSP_ADC_Channel_t channel, float *value_float)
{
    if (!value_float || channel >= BSP_ADC_CHANNEL_COUNT || !adc_state.is_initialized) {
        return false;
    }
    
    uint16_t raw_value = 0;
    
    // 获取原始值
    if (!BSP_ADC_GetChannelValue(channel, &raw_value)) {
        return false;
    }
    
    // 转换为物理单位
    switch (channel) {
        case BSP_ADC_CH_PRESSURE_SENSOR:
            *value_float = ADC_RawToPressure(raw_value);
            break;
        case BSP_ADC_CH_BATTERY_VOLTAGE:
            *value_float = ADC_RawToVoltage(raw_value);
            break;
        case BSP_ADC_CH_TEMP_BOARD:
        case BSP_ADC_CH_ONBOARD_TEMP:
            *value_float = ADC_RawToTemperature(raw_value);
            break;
        case BSP_ADC_CH_VREFINT:
            *value_float = ADC_RawToVoltage(raw_value);
            break;
        case BSP_ADC_CH_VBAT:
            *value_float = ADC_RawToVoltage(raw_value);
            break;
        default:
            return false;
    }
    
    return true;
}

/* ======================== 数据处理函数 ======================== */

bool BSP_ADC_ProcessData(const ADC_RawData_t *raw_data, ADC_ProcessedData_t *processed_data)
{
    if (!raw_data || !processed_data) {
        return false;
    }
    
    // 转换所有通道数据
    processed_data->pressure_pa = ADC_RawToPressure(raw_data->pressure_raw);
    processed_data->pressure_bar = processed_data->pressure_pa / 100000.0f;  // Pa to bar
    processed_data->temp_board = ADC_RawToTemperature(raw_data->temp_board_raw);
    processed_data->battery_voltage = ADC_RawToVoltage(raw_data->battery_volt_raw);
    processed_data->temp_chip = ADC_RawToTemperature(raw_data->temp_chip_raw);
    processed_data->vref_voltage = ADC_RawToVoltage(raw_data->vrefint_raw);
    processed_data->timestamp_ms = osKernelGetTickCount();
    processed_data->valid = true;
    
    // 更新统计信息
    if (processed_data->pressure_pa > adc_state.statistics.max_pressure) {
        adc_state.statistics.max_pressure = processed_data->pressure_pa;
    }
    if (processed_data->pressure_pa < adc_state.statistics.min_pressure) {
        adc_state.statistics.min_pressure = processed_data->pressure_pa;
    }
    adc_state.statistics.avg_pressure = (adc_state.statistics.avg_pressure * adc_state.statistics.total_samples + 
                                         processed_data->pressure_pa) / (adc_state.statistics.total_samples + 1);
    
    if (processed_data->battery_voltage > adc_state.statistics.max_battery_volt) {
        adc_state.statistics.max_battery_volt = processed_data->battery_voltage;
    }
    if (processed_data->battery_voltage < adc_state.statistics.min_battery_volt) {
        adc_state.statistics.min_battery_volt = processed_data->battery_voltage;
    }
    adc_state.statistics.avg_battery_volt = (adc_state.statistics.avg_battery_volt * adc_state.statistics.total_samples + 
                                             processed_data->battery_voltage) / (adc_state.statistics.total_samples + 1);
    
    adc_state.statistics.total_samples++;
    
    return true;
}

bool BSP_ADC_GetChannelAverage(BSP_ADC_Channel_t channel, uint16_t *avg_value, uint16_t sample_count)
{
    if (!avg_value || channel >= BSP_ADC_CHANNEL_COUNT || 
        sample_count == 0 || sample_count > BSP_ADC_DMA_BUFFER_SIZE ||
        !adc_state.is_initialized) {
        return false;
    }
    
    *avg_value = ADC_CalculateChannelAverage((uint8_t)channel, sample_count);
    return true;
}

/* ======================== 调试接口 ======================== */

bool BSP_ADC_GetStatistics(ADC_Statistics_t *stats)
{
    if (!stats) {
        return false;
    }
    
    // 禁用中断，保证数据一致性
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    memcpy(stats, &adc_state.statistics, sizeof(ADC_Statistics_t));
    
    __set_PRIMASK(primask);
    
    return true;
}

bool BSP_ADC_ResetStatistics(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    memset(&adc_state.statistics, 0, sizeof(ADC_Statistics_t));
    adc_state.statistics.adc_enabled = adc_state.is_initialized;
    adc_state.statistics.dma_running = adc_state.is_running;
    adc_state.statistics.min_pressure = 999999.0f;
    adc_state.statistics.max_pressure = 0.0f;
    adc_state.statistics.min_battery_volt = 999.0f;
    adc_state.statistics.max_battery_volt = 0.0f;
    
    __set_PRIMASK(primask);
    
    return true;
}

bool BSP_ADC_PrintDebugInfo(void)
{
    // 此函数需要UART接口，这里仅预留接口
    // 实际实现需要与UART驱动配合
    ADC_Statistics_t stats;
    ADC_ProcessedData_t processed;
    
    if (!BSP_ADC_GetStatistics(&stats) || !BSP_ADC_GetProcessedData(&processed)) {
        return false;
    }
    
    // 调试打印 (可通过UART输出)
    // printf("[ADC] Pressure: %.1f Pa (%.2f bar)\n", processed.pressure_pa, processed.pressure_bar);
    // printf("[ADC] Battery: %.2f V\n", processed.battery_voltage);
    // printf("[ADC] Temp: %.1f C\n", processed.temp_board);
    // ... 其他信息
    
    return true;
}

bool BSP_ADC_SetPressureCalibration(uint16_t adc_min, uint16_t adc_max, float press_min, float press_max)
{
    if (adc_min >= adc_max || press_min >= press_max) {
        return false;
    }
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    adc_state.calibration.pressure_adc_min = adc_min;
    adc_state.calibration.pressure_adc_max = adc_max;
    adc_state.calibration.pressure_min = press_min;
    adc_state.calibration.pressure_max = press_max;
    
    __set_PRIMASK(primask);
    
    return true;
}

bool BSP_ADC_SetBatteryCalibration(uint16_t adc_min, uint16_t adc_max, float volt_min, float volt_max)
{
    if (adc_min >= adc_max || volt_min >= volt_max) {
        return false;
    }
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    adc_state.calibration.battery_adc_min = adc_min;
    adc_state.calibration.battery_adc_max = adc_max;
    adc_state.calibration.battery_volt_min = volt_min;
    adc_state.calibration.battery_volt_max = volt_max;
    
    __set_PRIMASK(primask);
    
    return true;
}

bool BSP_ADC_GetCalibrationParams(
    uint16_t *pressure_adc_min, uint16_t *pressure_adc_max,
    float *pressure_min, float *pressure_max,
    uint16_t *battery_adc_min, uint16_t *battery_adc_max,
    float *battery_volt_min, float *battery_volt_max)
{
    if (!pressure_adc_min || !pressure_adc_max || !pressure_min || !pressure_max ||
        !battery_adc_min || !battery_adc_max || !battery_volt_min || !battery_volt_max) {
        return false;
    }
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    *pressure_adc_min = adc_state.calibration.pressure_adc_min;
    *pressure_adc_max = adc_state.calibration.pressure_adc_max;
    *pressure_min = adc_state.calibration.pressure_min;
    *pressure_max = adc_state.calibration.pressure_max;
    
    *battery_adc_min = adc_state.calibration.battery_adc_min;
    *battery_adc_max = adc_state.calibration.battery_adc_max;
    *battery_volt_min = adc_state.calibration.battery_volt_min;
    *battery_volt_max = adc_state.calibration.battery_volt_max;
    
    __set_PRIMASK(primask);
    
    return true;
}

/* ======================== DMA中断回调函数 ======================== */

void BSP_ADC_DMA_ConversionComplete(void)
{
    if (!adc_state.is_initialized) {
        return;
    }
    
    // 从DMA缓冲区提取最后一个完整的转换序列
    uint16_t last_offset = BSP_ADC_DMA_BUFFER_SIZE - BSP_ADC_CHANNEL_COUNT;
    ADC_ExtractConversionData(last_offset, &adc_state.current_raw_data);
    
    // 处理数据
    BSP_ADC_ProcessData(&adc_state.current_raw_data, &adc_state.current_processed_data);
    
    // 标记新数据可用
    adc_state.new_data_available = true;
    
    // 释放信号量通知任务有新数据
    if (adc_state.data_ready_semaphore != NULL) {
        osSemaphoreRelease(adc_state.data_ready_semaphore);
    }
    
    adc_state.statistics.dma_complete_count++;
}

void BSP_ADC_DMA_HalfConversion(void)
{
    if (!adc_state.is_initialized) {
        return;
    }
    
    // 从DMA缓冲区提取中点的转换序列 (用于实时处理)
    uint16_t mid_offset = BSP_ADC_DMA_BUFFER_SIZE / 2 - BSP_ADC_CHANNEL_COUNT;
    ADC_ExtractConversionData(mid_offset, &adc_state.current_raw_data);
    
    // 处理数据
    BSP_ADC_ProcessData(&adc_state.current_raw_data, &adc_state.current_processed_data);
    
    // 标记新数据可用
    adc_state.new_data_available = true;
    
    // 释放信号量通知任务有新数据
    if (adc_state.data_ready_semaphore != NULL) {
        osSemaphoreRelease(adc_state.data_ready_semaphore);
    }
}

void BSP_ADC_DMA_Error(void)
{
    if (adc_state.is_initialized) {
        adc_state.statistics.dma_error_count++;
    }
}

/* ======================== 私有函数实现 ======================== */

static void ADC_DMA_BufferInit(void)
{
    memset(adc_state.dma_buffer, 0, sizeof(adc_state.dma_buffer));
    memset(&adc_state.current_raw_data, 0, sizeof(ADC_RawData_t));
    memset(&adc_state.current_processed_data, 0, sizeof(ADC_ProcessedData_t));
}

static void ADC_ExtractConversionData(uint16_t buffer_offset, ADC_RawData_t *raw_data)
{
    if (!raw_data || buffer_offset + BSP_ADC_CHANNEL_COUNT > BSP_ADC_DMA_BUFFER_SIZE) {
        return;
    }
    
    // 按ADC转换序列顺序提取数据 (6 channels)
    raw_data->pressure_raw = adc_state.dma_buffer[buffer_offset + 0];
    raw_data->temp_board_raw = adc_state.dma_buffer[buffer_offset + 1];
    raw_data->battery_volt_raw = adc_state.dma_buffer[buffer_offset + 2];
    raw_data->temp_chip_raw = adc_state.dma_buffer[buffer_offset + 3];
    raw_data->vrefint_raw = adc_state.dma_buffer[buffer_offset + 4];
    raw_data->vbat_raw = adc_state.dma_buffer[buffer_offset + 5];
}

static uint16_t ADC_CalculateChannelAverage(uint8_t channel_index, uint16_t sample_count)
{
    if (channel_index >= BSP_ADC_CHANNEL_COUNT || sample_count == 0 || 
        sample_count > BSP_ADC_DMA_BUFFER_SIZE) {
        return 0;
    }
    
    uint32_t sum = 0;
    uint16_t count = 0;
    
    // 遍历DMA缓冲区中指定通道的所有采样
    for (uint16_t i = 0; i < sample_count; i++) {
        uint16_t idx = i * BSP_ADC_CHANNEL_COUNT + channel_index;
        if (idx < BSP_ADC_DMA_BUFFER_SIZE) {
            sum += adc_state.dma_buffer[idx];
            count++;
        }
    }
    
    if (count == 0) {
        return 0;
    }
    
    return (uint16_t)(sum / count);
}

static float ADC_RawToPressure(uint16_t raw_value)
{
    // 线性映射: raw_value -> pressure
    float pressure = adc_state.calibration.pressure_min + 
                     (float)(raw_value - adc_state.calibration.pressure_adc_min) * 
                     (adc_state.calibration.pressure_max - adc_state.calibration.pressure_min) / 
                     (float)(adc_state.calibration.pressure_adc_max - adc_state.calibration.pressure_adc_min);
    
    return pressure;
}

static float ADC_RawToVoltage(uint16_t raw_value)
{
    // 将ADC值转换为电压: voltage = (ADC / 4095) * VREF
    float voltage = (float)raw_value / (float)BSP_ADC_MAX_VALUE * BSP_ADC_VREF;
    
    return voltage;
}

static float ADC_RawToTemperature(uint16_t raw_value)
{
    // 温度传感器转换 (STM32F407):
    // T(℃) = (V25 - V) / Avg_Slope + 25
    // V = ADC_value * VREF / 4095
    
    float voltage = ADC_RawToVoltage(raw_value);
    float temperature = (BSP_ADC_TEMP_V25 - voltage) / BSP_ADC_TEMP_SLOPE + 25.0f;
    
    return temperature;
}
