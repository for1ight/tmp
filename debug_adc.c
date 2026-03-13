/**
 ******************************************************************************
 * @file    debug_adc.c
 * @brief   ADC调试接口实现 - 用于与上位机通信和数据验证
 ******************************************************************************
 */

#include "debug_adc.h"
#include "bsp_adc.h"
#include "usart.h"
#include <string.h>

/* ======================== 私有宏定义 ======================== */

/** 调试数据包头 */
#define DEBUG_PKT_HEADER    0xAA

/** 调试数据包尾 */
#define DEBUG_PKT_TAIL      0x55

/** 实时输出缓冲区大小 */
#define REALTIME_BUFFER_SIZE 256

/* ======================== 私有数据结构 ======================== */

/**
 * @brief 调试接口内部状态
 */
typedef struct {
    bool is_initialized;
    bool realtime_output_enabled;
    uint8_t realtime_output_type;
    uint32_t packet_count;
    uint32_t error_count;
} DebugADC_State_t;

/* ======================== 私有全局变量 ======================== */

static DebugADC_State_t debug_state = {0};
static uint8_t realtime_buffer[REALTIME_BUFFER_SIZE];

/* ======================== 私有函数声明 ======================== */

/**
 * @brief 发送数据包通过UART
 */
static bool DebugADC_SendPacket(const uint8_t *data, uint16_t length);

/**
 * @brief 处理"获取原始ADC值"命令
 */
static bool DebugADC_Cmd_GetRawData(uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"获取处理后数据"命令
 */
static bool DebugADC_Cmd_GetProcessedData(uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"获取统计信息"命令
 */
static bool DebugADC_Cmd_GetStatistics(uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"重置统计信息"命令
 */
static bool DebugADC_Cmd_ResetStatistics(uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"获取标定参数"命令
 */
static bool DebugADC_Cmd_GetCalibration(uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"设置标定参数"命令
 */
static bool DebugADC_Cmd_SetCalibration(const uint8_t *data, uint16_t length,
                                         uint8_t *response, uint16_t *resp_len);

/**
 * @brief 处理"查询ADC状态"命令
 */
static bool DebugADC_Cmd_GetStatus(uint8_t *response, uint16_t *resp_len);

/* ======================== 初始化函数 ======================== */

bool DebugADC_Init(void)
{
    if (debug_state.is_initialized) {
        return true;
    }
    
    memset(&debug_state, 0, sizeof(DebugADC_State_t));
    debug_state.is_initialized = true;
    debug_state.realtime_output_enabled = false;
    
    return true;
}

/* ======================== 命令处理函数 ======================== */

bool DebugADC_ProcessCommand(uint8_t command, const uint8_t *data, uint16_t length,
                              uint8_t *response, uint16_t *resp_len)
{
    if (!response || !resp_len || !debug_state.is_initialized) {
        return false;
    }
    
    bool result = false;
    
    switch (command) {
        case DEBUG_CMD_GET_ADC_RAW:
            result = DebugADC_Cmd_GetRawData(response, resp_len);
            break;
            
        case DEBUG_CMD_GET_ADC_PROCESSED:
            result = DebugADC_Cmd_GetProcessedData(response, resp_len);
            break;
            
        case DEBUG_CMD_GET_STATISTICS:
            result = DebugADC_Cmd_GetStatistics(response, resp_len);
            break;
            
        case DEBUG_CMD_RESET_STATISTICS:
            result = DebugADC_Cmd_ResetStatistics(response, resp_len);
            break;
            
        case DEBUG_CMD_GET_CALIBRATION:
            result = DebugADC_Cmd_GetCalibration(response, resp_len);
            break;
            
        case DEBUG_CMD_SET_CALIBRATION:
            result = DebugADC_Cmd_SetCalibration(data, length, response, resp_len);
            break;
            
        case DEBUG_CMD_ADC_STATUS:
            result = DebugADC_Cmd_GetStatus(response, resp_len);
            break;
            
        default:
            *resp_len = 0;
            result = false;
            break;
    }
    
    if (!result) {
        debug_state.error_count++;
    }
    
    return result;
}

bool DebugADC_BuildResponsePacket(DebugADC_ResponsePacket_t *packet,
                                   uint8_t command, const uint8_t *data, uint16_t length)
{
    if (!packet || length > sizeof(packet->data)) {
        return false;
    }
    
    packet->header = DEBUG_PKT_HEADER;
    packet->command = command;
    packet->length = length;
    
    if (data && length > 0) {
        memcpy(packet->data, data, length);
    }
    
    // 计算校验和 (command + length + data)
    uint8_t checksum = command;
    checksum ^= (length >> 8);
    checksum ^= (length & 0xFF);
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= packet->data[i];
    }
    packet->checksum = checksum;
    
    packet->tail = DEBUG_PKT_TAIL;
    
    return true;
}

uint8_t DebugADC_CalculateChecksum(const uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

/* ======================== 上位机通信接口 ======================== */

bool DebugADC_SendRawData(void)
{
    ADC_RawData_t raw_data;
    
    if (!BSP_ADC_GetRawData(&raw_data)) {
        return false;
    }
    
    DebugADC_RawDataPacket_t pkt;
    pkt.pressure_raw = raw_data.pressure_raw;
    pkt.temp_board_raw = raw_data.temp_board_raw;
    pkt.battery_volt_raw = raw_data.battery_volt_raw;
    pkt.temp_chip_raw = raw_data.temp_chip_raw;
    pkt.vrefint_raw = raw_data.vrefint_raw;
    pkt.vbat_raw = raw_data.vbat_raw;
    pkt.timestamp_ms = (uint32_t)HAL_GetTick();
    
    DebugADC_ResponsePacket_t resp_pkt;
    DebugADC_BuildResponsePacket(&resp_pkt, DEBUG_CMD_GET_ADC_RAW, 
                                  (uint8_t *)&pkt, sizeof(pkt));
    
    return DebugADC_SendPacket((uint8_t *)&resp_pkt, sizeof(resp_pkt));
}

bool DebugADC_SendProcessedData(void)
{
    ADC_ProcessedData_t processed_data;
    
    if (!BSP_ADC_GetProcessedData(&processed_data)) {
        return false;
    }
    
    DebugADC_ProcessedDataPacket_t pkt;
    pkt.pressure_pa = processed_data.pressure_pa;
    pkt.pressure_bar = processed_data.pressure_bar;
    pkt.temp_board = processed_data.temp_board;
    pkt.battery_voltage = processed_data.battery_voltage;
    pkt.temp_chip = processed_data.temp_chip;
    pkt.vref_voltage = processed_data.vref_voltage;
    pkt.timestamp_ms = processed_data.timestamp_ms;
    
    DebugADC_ResponsePacket_t resp_pkt;
    DebugADC_BuildResponsePacket(&resp_pkt, DEBUG_CMD_GET_ADC_PROCESSED,
                                  (uint8_t *)&pkt, sizeof(pkt));
    
    return DebugADC_SendPacket((uint8_t *)&resp_pkt, sizeof(resp_pkt));
}

bool DebugADC_SendStatistics(void)
{
    ADC_Statistics_t stats;
    
    if (!BSP_ADC_GetStatistics(&stats)) {
        return false;
    }
    
    DebugADC_StatisticsPacket_t pkt;
    pkt.total_samples = stats.total_samples;
    pkt.dma_complete_count = stats.dma_complete_count;
    pkt.dma_error_count = stats.dma_error_count;
    pkt.avg_pressure = stats.avg_pressure;
    pkt.min_pressure = stats.min_pressure;
    pkt.max_pressure = stats.max_pressure;
    pkt.avg_battery_volt = stats.avg_battery_volt;
    pkt.min_battery_volt = stats.min_battery_volt;
    pkt.max_battery_volt = stats.max_battery_volt;
    pkt.adc_enabled = stats.adc_enabled ? 1 : 0;
    pkt.dma_running = stats.dma_running ? 1 : 0;
    
    DebugADC_ResponsePacket_t resp_pkt;
    DebugADC_BuildResponsePacket(&resp_pkt, DEBUG_CMD_GET_STATISTICS,
                                  (uint8_t *)&pkt, sizeof(pkt));
    
    return DebugADC_SendPacket((uint8_t *)&resp_pkt, sizeof(resp_pkt));
}

void DebugADC_UARTReceiveCallback(const uint8_t *data, uint16_t length)
{
    if (!data || length < 2) {
        return;
    }
    
    // 检查数据包格式
    if (data[0] != DEBUG_PKT_HEADER) {
        return;
    }
    
    uint8_t command = data[1];
    uint8_t response[256];
    uint16_t resp_len = 0;
    
    // 提取命令数据 (如果有)
    const uint8_t *cmd_data = NULL;
    uint16_t cmd_length = 0;
    if (length > 2) {
        cmd_data = &data[2];
        cmd_length = length - 2;
    }
    
    // 处理命令
    DebugADC_ProcessCommand(command, cmd_data, cmd_length, response, &resp_len);
}

/* ======================== 实时监测接口 ======================== */

bool DebugADC_EnableRealtimeOutput(uint8_t output_type)
{
    if (output_type > 1) {
        return false;
    }
    
    debug_state.realtime_output_enabled = true;
    debug_state.realtime_output_type = output_type;
    
    return true;
}

void DebugADC_DisableRealtimeOutput(void)
{
    debug_state.realtime_output_enabled = false;
}

bool DebugADC_IsRealtimeOutputEnabled(void)
{
    return debug_state.realtime_output_enabled;
}

/* ======================== 私有函数实现 ======================== */

static bool DebugADC_SendPacket(const uint8_t *data, uint16_t length)
{
    if (!data || length == 0) {
        return false;
    }
    
    // 通过UART发送数据
    // 这里需要与实际的UART驱动配合
    // HAL_UART_Transmit(&huart_debug, (uint8_t *)data, length, 100);
    
    // 暂时使用占位符
    (void)data;
    (void)length;
    
    debug_state.packet_count++;
    return true;
}

static bool DebugADC_Cmd_GetRawData(uint8_t *response, uint16_t *resp_len)
{
    ADC_RawData_t raw_data;
    
    if (!BSP_ADC_GetRawData(&raw_data)) {
        *resp_len = 0;
        return false;
    }
    
    DebugADC_RawDataPacket_t *pkt = (DebugADC_RawDataPacket_t *)response;
    pkt->pressure_raw = raw_data.pressure_raw;
    pkt->temp_board_raw = raw_data.temp_board_raw;
    pkt->battery_volt_raw = raw_data.battery_volt_raw;
    pkt->temp_chip_raw = raw_data.temp_chip_raw;
    pkt->vrefint_raw = raw_data.vrefint_raw;
    pkt->vbat_raw = raw_data.vbat_raw;
    pkt->timestamp_ms = (uint32_t)HAL_GetTick();
    
    *resp_len = sizeof(DebugADC_RawDataPacket_t);
    return true;
}

static bool DebugADC_Cmd_GetProcessedData(uint8_t *response, uint16_t *resp_len)
{
    ADC_ProcessedData_t processed_data;
    
    if (!BSP_ADC_GetProcessedData(&processed_data)) {
        *resp_len = 0;
        return false;
    }
    
    DebugADC_ProcessedDataPacket_t *pkt = (DebugADC_ProcessedDataPacket_t *)response;
    pkt->pressure_pa = processed_data.pressure_pa;
    pkt->pressure_bar = processed_data.pressure_bar;
    pkt->temp_board = processed_data.temp_board;
    pkt->battery_voltage = processed_data.battery_voltage;
    pkt->temp_chip = processed_data.temp_chip;
    pkt->vref_voltage = processed_data.vref_voltage;
    pkt->timestamp_ms = processed_data.timestamp_ms;
    
    *resp_len = sizeof(DebugADC_ProcessedDataPacket_t);
    return true;
}

static bool DebugADC_Cmd_GetStatistics(uint8_t *response, uint16_t *resp_len)
{
    ADC_Statistics_t stats;
    
    if (!BSP_ADC_GetStatistics(&stats)) {
        *resp_len = 0;
        return false;
    }
    
    DebugADC_StatisticsPacket_t *pkt = (DebugADC_StatisticsPacket_t *)response;
    pkt->total_samples = stats.total_samples;
    pkt->dma_complete_count = stats.dma_complete_count;
    pkt->dma_error_count = stats.dma_error_count;
    pkt->avg_pressure = stats.avg_pressure;
    pkt->min_pressure = stats.min_pressure;
    pkt->max_pressure = stats.max_pressure;
    pkt->avg_battery_volt = stats.avg_battery_volt;
    pkt->min_battery_volt = stats.min_battery_volt;
    pkt->max_battery_volt = stats.max_battery_volt;
    pkt->adc_enabled = stats.adc_enabled ? 1 : 0;
    pkt->dma_running = stats.dma_running ? 1 : 0;
    
    *resp_len = sizeof(DebugADC_StatisticsPacket_t);
    return true;
}

static bool DebugADC_Cmd_ResetStatistics(uint8_t *response, uint16_t *resp_len)
{
    bool result = BSP_ADC_ResetStatistics();
    
    if (result) {
        response[0] = 0x01;  // 成功标志
        *resp_len = 1;
    } else {
        response[0] = 0x00;  // 失败标志
        *resp_len = 1;
    }
    
    return result;
}

static bool DebugADC_Cmd_GetCalibration(uint8_t *response, uint16_t *resp_len)
{
    DebugADC_CalibrationPacket_t *pkt = (DebugADC_CalibrationPacket_t *)response;
    
    bool result = BSP_ADC_GetCalibrationParams(
        &pkt->pressure_adc_min, &pkt->pressure_adc_max,
        &pkt->pressure_min, &pkt->pressure_max,
        &pkt->battery_adc_min, &pkt->battery_adc_max,
        &pkt->battery_volt_min, &pkt->battery_volt_max
    );
    
    if (result) {
        *resp_len = sizeof(DebugADC_CalibrationPacket_t);
    } else {
        *resp_len = 0;
    }
    
    return result;
}

static bool DebugADC_Cmd_SetCalibration(const uint8_t *data, uint16_t length,
                                         uint8_t *response, uint16_t *resp_len)
{
    if (length < sizeof(DebugADC_CalibrationPacket_t)) {
        response[0] = 0x00;
        *resp_len = 1;
        return false;
    }
    
    const DebugADC_CalibrationPacket_t *pkt = (const DebugADC_CalibrationPacket_t *)data;
    
    // 设置压力标定参数
    BSP_ADC_SetPressureCalibration(pkt->pressure_adc_min, pkt->pressure_adc_max,
                                   pkt->pressure_min, pkt->pressure_max);
    
    // 设置电池标定参数
    BSP_ADC_SetBatteryCalibration(pkt->battery_adc_min, pkt->battery_adc_max,
                                  pkt->battery_volt_min, pkt->battery_volt_max);
    
    response[0] = 0x01;  // 成功标志
    *resp_len = 1;
    
    return true;
}

static bool DebugADC_Cmd_GetStatus(uint8_t *response, uint16_t *resp_len)
{
    response[0] = BSP_ADC_IsRunning() ? 0x01 : 0x00;
    *resp_len = 1;
    
    return true;
}
