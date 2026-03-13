/**
 ******************************************************************************
 * @file    task_cmd.c
 * @brief   上位机指令接收任务 - 使用 DMA + IDLE 中断
 * @note    处理不定长指令帧,解析后写入 DataHub
 ******************************************************************************
 */

#include "task_cmd.h"
#include "data_hub.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

/* ======================== 配置参数 ======================== */
#define CMD_RX_BUF_SIZE     256     // 接收缓冲区大小
#define CMD_TIMEOUT_MS      1000    // 指令超时时间

/* ======================== 私有变量 ======================== */
static uint8_t g_RxBuffer[CMD_RX_BUF_SIZE];
static volatile uint16_t g_RxLength = 0;
static osSemaphoreId_t g_RxSemaphore;

/* ======================== 私有函数 ======================== */

/**
 * @brief 计算 CRC16
 */
static uint16_t CalcCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t j;
    
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 解析指令帧
 */
static bool ParseCmdFrame(const uint8_t *data, uint16_t len, Command_t *cmd)
{
    CmdFrame_t *frame;
    uint16_t calc_crc;
    
    if (len < sizeof(CmdFrame_t)) return false;
    
    frame = (CmdFrame_t*)data;
    
    // 校验帧头和帧尾
    if (frame->header[0] != 0xBB || frame->header[1] != 0x66 || frame->tail != 0x0A) {
        return false;
    }
    
    // 校验长度
    if (frame->length != sizeof(CmdFrame_t) - 4) {
        return false;
    }
    
    // 校验 CRC
    calc_crc = CalcCRC16(data, sizeof(CmdFrame_t) - 3);
    if (frame->checksum != calc_crc) {
        return false;
    }
    
    // 解析数据
    cmd->mode = frame->mode;
    cmd->pitch_cmd = frame->pitch_cmd;
    cmd->yaw_cmd = frame->yaw_cmd;
    cmd->roll_cmd = frame->roll_cmd;
    cmd->thrust_cmd[0] = frame->thrust_cmd[0];
    cmd->thrust_cmd[1] = frame->thrust_cmd[1];
    
    cmd->target_depth = frame->target_depth;
    cmd->target_pitch = frame->target_pitch;
    cmd->target_yaw = frame->target_yaw;
    
    cmd->valve_open = frame->valve_open;
    
    cmd->timestamp = HAL_GetTick();
    cmd->seq = frame->seq;
    
    return true;
}

/* ======================== 中断回调 ======================== */

/**
 * @brief UART 空闲中断回调
 * @note 在 stm32f4xx_it.c 的 USART1_IRQHandler 中调用
 */
void USART1_IDLE_Callback(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        // 停止 DMA 并计算接收长度
        HAL_UART_DMAStop(&huart1);
        g_RxLength = CMD_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        
        // 释放信号量通知任务
        osSemaphoreRelease(g_RxSemaphore);
        
        // 重新启动 DMA 接收
        HAL_UART_Receive_DMA(&huart1, g_RxBuffer, CMD_RX_BUF_SIZE);
    }
}

/* ======================== 任务入口 ======================== */

static uint32_t count=0;
void StartCmdTask(void *argument)
{
    Command_t cmd;
    uint32_t last_cmd_time;
    count++;
    // 创建信号量
    g_RxSemaphore = osSemaphoreNew(1, 0, NULL);
    
    // 启动 DMA 接收 (循环模式)
    
    last_cmd_time = 0;
    
    HAL_UART_Receive_DMA(&huart1, g_RxBuffer, CMD_RX_BUF_SIZE);
    
    // 使能 UART 空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
    for(;;)
    {
				#ifdef SIMULATOR_MODE
					printf("%s\n", __FUNCTION__);
				#endif
        // 等待接收完成信号
        if (osSemaphoreAcquire(g_RxSemaphore, CMD_TIMEOUT_MS) == osOK) {
            // 解析指令
            if (ParseCmdFrame(g_RxBuffer, g_RxLength, &cmd)) {
                // 写入 DataHub
                DataHub_WriteCommand(&cmd);
                last_cmd_time = HAL_GetTick();
                
                // 可选: 发送 ACK 确认
                // uint8_t ack[] = {0xAA, 0x55, 0x00, 0x01, 0x00};
                // HAL_UART_Transmit(&huart1, ack, sizeof(ack), 10);
            }
        } else {
            // 超时: 检查是否需要进入安全模式
            if (HAL_GetTick() - last_cmd_time > CMD_TIMEOUT_MS * 5) {
                // 5 秒无指令,切换到待机模式
                cmd.mode = 0;
                cmd.timestamp = HAL_GetTick();
                DataHub_WriteCommand(&cmd);
            }
        }
    }
}

/* ======================== UART 中断处理 ======================== */
// 需要在 stm32f4xx_it.c 中添加:
/*
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
    USART1_IDLE_Callback();  // 调用空闲中断处理
}
*/
