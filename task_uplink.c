/**
 ******************************************************************************
 * @file    task_uplink.c
 * @brief   上位机数据上传任务 - 通过 TTL 串口发送状态
 * @note    使用 DMA 传输,定期打包发送所有传感器和执行器状态
 ******************************************************************************
 */

#include "task_uplink.h"
#include "data_hub.h"
#include "bsp_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

/* ======================== 配置参数 ======================== */
#define UPLINK_FREQ_HZ      100     // 上传频率 100Hz (10ms 周期)
#define UPLINK_PERIOD_MS    (1000 / UPLINK_FREQ_HZ)

/* ======================== 私有变量 ======================== */
static uint8_t g_TxBuffer[sizeof(UplinkFrame_t)];

/* ======================== 私有函数 ======================== */

/**
 * @brief 计算 CRC16 校验 (CCITT)
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
 * @brief 打包上行数据帧 (实现)
 */
static void PackUplinkFrame(UplinkFrame_t *frame)
{
    SensorData_t sensor;
    TelemetryData_t telem;
    ControlOutput_t output;
    int i;
    
    // 读取所有数据
    DataHub_ReadSensor(&sensor);
    DataHub_ReadTelemetry(&telem);
    DataHub_ReadOutput(&output);
    
    // 帧头
    frame->header[0] = 0xAA;
    frame->header[1] = 0x55;
    frame->length = sizeof(UplinkFrame_t) - 6;  // 减去帧头和长度字段
    frame->timestamp = HAL_GetTick();
    
    // 传感器数据
    frame->roll = sensor.roll;
    frame->pitch = sensor.pitch;
    frame->yaw = sensor.yaw;
    frame->depth = sensor.depth_m;
    frame->pressure_raw = sensor.pressure_raw;
    
    // 执行器反馈
    for (i = 0; i < 3; i++) {
        frame->servo[i].angle = telem.servo[i].angle_deg;
        frame->servo[i].current = telem.servo[i].current_ma;
        frame->servo[i].online = telem.servo[i].online;
    }
    
    for (i = 0; i < 2; i++) {
        frame->esc[i].rpm = telem.esc[i].rpm;
        frame->esc[i].voltage = telem.esc[i].voltage_mv;
        frame->esc[i].current = telem.esc[i].current_ma;
        frame->esc[i].temp = telem.esc[i].temp_c;
        frame->esc[i].online = telem.esc[i].online;
    }
    
    // 控制输出
    for (i = 0; i < 3; i++) {
        frame->servo_pwm[i] = output.servo_pwm[i];
    }
    for (i = 0; i < 2; i++) {
        frame->esc_pwm[i] = output.esc_pwm[i];
    }
    frame->valve_state = output.valve_state.val;
    frame->ctrl_mode = output.ctrl_mode;
    
    // 系统状态 (使用系统时间和默认错误码)
    frame->uptime = osKernelGetTickCount() / 1000;  // 转换为秒
    frame->error_code = 0;  // 无错误
    
    // 计算校验和 (不包含帧尾和校验和本身)
    frame->checksum = CalcCRC16((uint8_t*)frame, sizeof(UplinkFrame_t) - 3);
    frame->tail = 0x0D;
}

/* ======================== 任务入口 ======================== */

void StartUplinkTask(void *argument)
{
    UplinkFrame_t *frame = (UplinkFrame_t*)g_TxBuffer;
    
    for(;;)
    {
				#ifdef SIMULATOR_MODE
					printf("%s\n", __FUNCTION__);
				#endif
        // 打包数据
        PackUplinkFrame(frame);
        
        // 通过 DMA 发送 (USART1)
        HAL_UART_Transmit_DMA(&huart1, g_TxBuffer, sizeof(UplinkFrame_t));
        
        // 等待发送完成 (可选:通过信号量等待 DMA 完成中断)
        // 此处简化为延时
        // osDelay(5);
        
        // // 固定周期延时
        // osDelay(UPLINK_PERIOD_MS - 5);
    }
}

/* ======================== 调试接口 ======================== */
