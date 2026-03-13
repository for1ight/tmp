/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @brief   多路 UART 驱动实现 - UART3/4 DMA自动接收, UART5 单总线查询
 * @note    UART3/4: 电机自动反馈 (DMA接收)
 *          UART5: 舵机单总线 (主动查询, 需要模式切换)
 ******************************************************************************
 */

#include "bsp_uart.h"
#include "usart.h"
#include "dma.h"
#include "cmsis_os.h"
#include <cstdint>
#include <string.h>
#include <stdbool.h>
#include "common_types.h"
/* ======================== 私有变量 ======================== */

// UART3 (电机4) - DMA自动接收
static uint8_t g_UART3_RxBuffer[UART3_RX_BUF_SIZE];
volatile uint16_t g_UART3_RxCount = 0;  // 全局可访问

// UART4 (电机3) - DMA自动接收
static uint8_t g_UART4_RxBuffer[UART4_RX_BUF_SIZE];
volatile uint16_t g_UART4_RxCount = 0;  // 全局可访问

// UART5 (舵机) - 单总线查询
static uint8_t g_UART5_RxBuffer[UART5_RX_BUF_SIZE];
osSemaphoreId_t g_UART5_RxCompleteSem;  // 全局可访问
static volatile bool g_UART5_IsTransmitting = false;

/* ======================== UART5 单总线模式切换 ======================== */

/**
 * @brief 切换UART5为发送模式 (半双工)
 */
static void UART5_SwitchToTxMode(void)
{
    // 禁用接收
    __HAL_UART_DISABLE(&huart5);
    CLEAR_BIT(huart5.Instance->CR1, USART_CR1_RE);
    
    // 启用发送
    SET_BIT(huart5.Instance->CR1, USART_CR1_TE);
    __HAL_UART_ENABLE(&huart5);
    
    g_UART5_IsTransmitting = true;
}

/**
 * @brief 切换UART5为接收模式 (半双工)
 */
static void UART5_SwitchToRxMode(void)
{
    // 等待发送完成
    while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {
        osDelay(1);
    }
    
    // 禁用发送
    __HAL_UART_DISABLE(&huart5);
    CLEAR_BIT(huart5.Instance->CR1, USART_CR1_TE);
    
    // 清除接收缓冲和标志 (防止使用旧数据)
    huart5.Instance->SR = 0;
    
    // 启用接收
    SET_BIT(huart5.Instance->CR1, USART_CR1_RE);
    __HAL_UART_ENABLE(&huart5);
    
    g_UART5_IsTransmitting = false;
}

/* ======================== 初始化函数 ======================== */

void BSP_MultiUART_Init(void)
{
    // 创建UART5接收完成信号量
    g_UART5_RxCompleteSem = osSemaphoreNew(1, 0, NULL);
    
    // UART3: 启动DMA接收 (电机4自动反馈)
    HAL_UART_Receive_DMA(&huart3, g_UART3_RxBuffer, UART3_RX_BUF_SIZE);
    
    // UART4: 启动DMA接收 (电机3自动反馈)
    HAL_UART_Receive_DMA(&huart4, g_UART4_RxBuffer, UART4_RX_BUF_SIZE);
    
    // UART5: 初始化为接收模式 (单总线)
    // 注: CubeMX中应已配置 UART5 为半双工模式 (CR3.HDSEL=1)
    UART5_SwitchToRxMode();
}

/* ======================== UART3 API (电机4) ======================== */

uint8_t* BSP_UART3_GetRxBuffer(void)
{
    return g_UART3_RxBuffer;
}

uint16_t BSP_UART3_GetRxCount(void)
{
    // 计算已接收字节数
    // UART3接收完成时会触发回调，计算实际接收的字节数
    // 如果使用环形DMA，需要从DMA计数器计算
    // 临时方案: 返回固定的电机反馈包大小
    if (g_UART3_RxCount > 0) {
        return g_UART3_RxCount;
    }
    return 0;
}

void BSP_UART3_ClearRxBuffer(void)
{
    // 停止DMA并重新启动
    HAL_UART_DMAStop(&huart3);
    memset(g_UART3_RxBuffer, 0, UART3_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart3, g_UART3_RxBuffer, UART3_RX_BUF_SIZE);
    g_UART3_RxCount = 0;
}

/* ======================== UART4 API (电机3) ======================== */

uint8_t* BSP_UART4_GetRxBuffer(void)
{
    return g_UART4_RxBuffer;
}

uint16_t BSP_UART4_GetRxCount(void)
{
    // 计算已接收字节数
    // UART4接收完成时会触发回调，计算实际接收的字节数
    // 如果使用环形DMA，需要从DMA计数器计算
    // 临时方案: 返回固定的电机反馈包大小
    if (g_UART4_RxCount > 0) {
        return g_UART4_RxCount;
    }
    return 0;
}

void BSP_UART4_ClearRxBuffer(void)
{
    // 停止DMA并重新启动
    HAL_UART_DMAStop(&huart4);
    memset(g_UART4_RxBuffer, 0, UART4_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart4, g_UART4_RxBuffer, UART4_RX_BUF_SIZE);
    g_UART4_RxCount = 0;
}

/* ======================== UART5 API (舵机) ======================== */

/**
 * @brief 计算舵机命令的校验和 (求和 -> 取低8位 -> 与0xFF异或)
 * @note  校验逻辑: (SUM & 0xFF) ^ 0xFF
 */
static uint8_t Servo_CalcChecksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;
    uint16_t i;
    
    // 1. 累加所有数值
    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    
    // 2. 取低8位，与0xFF异或
    return (uint8_t)((sum & 0xFF) ^ 0xFF);
}

HAL_StatusTypeDef BSP_Servo_SetID(uint8_t current_id, uint8_t target_id)
{
    ServoInitCmd_t cmd;
    ServoResponse_t resp;
    HAL_StatusTypeDef status;
    uint32_t start_tick;
    
    // 构造设置ID命令 (根据图2协议)
    // 注: 具体包头、命令字需根据实际舵机协议调整
    cmd.header = 0xFF;           // 包头
    cmd.id = current_id;         // 当前ID (0 = 未编程)
    cmd.cmd = 0x03;              // 设置ID命令
    cmd.param = target_id;       // 目标ID
    cmd.checksum = Servo_CalcChecksum((uint8_t*)&cmd, sizeof(cmd) - 1);
    
    // 1. 切换到发送模式
    UART5_SwitchToTxMode();
    
    // 2. 使用DMA发送命令
    status = HAL_UART_Transmit_DMA(&huart5, (uint8_t*)&cmd, sizeof(cmd));
    if (status != HAL_OK) {
        UART5_SwitchToRxMode();
        return status;
    }
    
    // 3. 等待发送完成
    start_tick = HAL_GetTick();
    while (g_UART5_IsTransmitting) {
        if (HAL_GetTick() - start_tick > UART5_QUERY_TIMEOUT_MS) {
            UART5_SwitchToRxMode();
            return HAL_TIMEOUT;
        }
        osDelay(1);
    }
    
    // 4. 切换到接收模式
    osDelay(2);  // 短延迟等待舵机响应
    UART5_SwitchToRxMode();
    
    // 5. 使用DMA接收响应
    status = HAL_UART_Receive_DMA(&huart5, (uint8_t*)&resp, sizeof(resp));
    if (status != HAL_OK) {
        return status;
    }
    
    // 6. 等待接收完成
    if (osSemaphoreAcquire(g_UART5_RxCompleteSem, UART5_QUERY_TIMEOUT_MS) == osOK) {
        // 校验响应
        uint8_t calc_checksum = Servo_CalcChecksum((uint8_t*)&resp, sizeof(resp) - 1);
        if (resp.header == 0xFF && resp.checksum == calc_checksum) {
            return HAL_OK;
        }
        return HAL_ERROR;
    }
    
    // 超时
    HAL_UART_DMAStop(&huart5);
    return HAL_TIMEOUT;
}
//TODO: 检查一下这个函数
HAL_StatusTypeDef BSP_Servo_QueryAngle(uint8_t servo_id, float *angle)
{
    ServoQueryCmd_t cmd;
    ServoResponse_t resp;
    HAL_StatusTypeDef status;
    uint32_t start_tick;
    uint16_t angle_raw;
    
    if (angle == NULL) {
        return HAL_ERROR;
    }
    
    // 构造查询位置命令 (根据图3协议)
    cmd.header = 0xFF;           // 包头
    cmd.device_id = servo_id;           // 舵机ID
    cmd.cmd = 0x01;              // 查询位置命令
    cmd.checksum = Servo_CalcChecksum((uint8_t*)&cmd, sizeof(cmd) - 1);
    
    // 1. 切换到发送模式
    UART5_SwitchToTxMode();
    
    // 2. 使用DMA发送命令
    status = HAL_UART_Transmit_DMA(&huart5, (uint8_t*)&cmd, sizeof(cmd));
    if (status != HAL_OK) {
        UART5_SwitchToRxMode();
        return status;
    }
    
    // 3. 等待发送完成
    start_tick = HAL_GetTick();
    while (g_UART5_IsTransmitting) {
        if (HAL_GetTick() - start_tick > UART5_QUERY_TIMEOUT_MS) {
            UART5_SwitchToRxMode();
            return HAL_TIMEOUT;
        }
        osDelay(1);
    }
    
    // 4. 切换到接收模式
    osDelay(2);  // 短延迟等待舵机响应
    UART5_SwitchToRxMode();
    
    // 5. 使用DMA接收响应
    status = HAL_UART_Receive_DMA(&huart5, (uint8_t*)&resp, sizeof(resp));
    if (status != HAL_OK) {
        return status;
    }
    
    // 6. 等待接收完成
    if (osSemaphoreAcquire(g_UART5_RxCompleteSem, UART5_QUERY_TIMEOUT_MS) == osOK) {
        // 校验响应
        uint8_t calc_checksum = Servo_CalcChecksum((uint8_t*)&resp, sizeof(resp) - 1);
        if (resp.header == 0xFF && resp.id == servo_id && resp.checksum == calc_checksum) {
            // 解析角度 (高字节在前)
            angle_raw = ((uint16_t)resp.angle_h << 8) | resp.angle_l;
            *angle = angle_raw / 10.0f;  // 转换为度数
            return HAL_OK;
        }
        return HAL_ERROR;
    }
    
    // 超时
    HAL_UART_DMAStop(&huart5);
    return HAL_TIMEOUT;
}

/* ======================== 校验函数 ======================== */

/**
 * @brief 计算CRC8校验 (电机反馈数据包)
 * @note  多项式: x^8+x^7+x^5+x^4+1 (0xF7)
 */

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++) {
        if (crc_u & 0x80) {
            crc_u = (crc_u << 1) ^ 0xF7;
        } else {
            crc_u <<= 1;
        }
    }
    return crc_u;
}
uint8_t BSP_CalcChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    int j;
    
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            crc = update_crc8(data[i], crc);
        }
    }
    return crc;
}

/**
 * @brief 计算CRC16校验码 (标准CCITT)
 * @note  多项式: x^16+x^12+x^5+1 (0x1021)
 */
uint16_t BSP_CalcCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    int j;
    
    for (i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

/* ======================== HAL 回调函数 ======================== */

/**
 * @brief UART DMA 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        g_UART5_IsTransmitting = false;
    }
}

/**
 * @brief UART 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        // 清除错误标志并重新初始化
        huart5.Instance->SR = 0;
        UART5_SwitchToRxMode();
    }
}
