/**
 ******************************************************************************
 * @file    task_telemetry.c
 * @brief   遥测数据采集任务 - UART3/4电机反馈 + UART5舵机查询
 * @note    UART3: 接收电机4自动反馈 (DMA)
 *          UART4: 接收电机3自动反馈 (DMA)
 *          UART5: 舵机5单总线查询 (初始化ID + 定期查询位置)
 ******************************************************************************
 */

#include "task_telemetry.h"
#include "data_hub.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include <cstdint>
#include <string.h>
#include <stdio.h>

/* ======================== 配置参数 ======================== */
#define SERVO_QUERY_INTERVAL_MS   100     // 舵机查询周期 (10Hz)
#define MOTOR_PARSE_CHECK_MS      50      // 电机数据解析检查周期
#define SERVO_ID_INIT_TIMEOUT_MS  5000    // 舵机初始化超时
#define MOTOR_DATA_TIMEOUT_MS     1000    // 电机数据超时 (1秒)

// 舵机配置
#define SERVO_COUNT               3       // 舵机数量
#define SERVO_BASE_ID             1       // 舵机起始ID号

/* ======================== 私有变量 ======================== */

// 电机数据缓存
static MotorData_t g_MotorData[2] = {0};  // [0]=电机4, [1]=电机3
static uint8_t g_ServoOnline[3] = {0};


/* ======================== 电机数据解析函数 ======================== */

/**
 * @brief 解析电机反馈数据包 (10字节, 包含CRC8校验)
 * @note  协议见图1:
 *        Byte0: 温度 (℃)
 *        Byte1-2: 电压 (0.01V, 高字节在前)
 *        Byte3-4: 电流 (0.01A, 高字节在前)
 *        Byte5-6: 电量 (mAh, 高字节在前)
 *        Byte7-8: 转速 (100RPM, 高字节在前)
 *        Byte9: CRC8校验
 */
bool TelemetryTask_ParseMotorFeedback(const uint8_t *data, uint16_t len, MotorData_t *motor_data)
{
    MotorFeedback_t *feedback;
    uint8_t calc_crc;
    
    if (!data || len < sizeof(MotorFeedback_t) || !motor_data) {
        return false;
    }
    
    feedback = (MotorFeedback_t *)data;
    
    // 验证CRC8校验
    calc_crc = BSP_CalcChecksum((uint8_t *)feedback, sizeof(MotorFeedback_t) - 1);
    if (calc_crc != feedback->crc) {
        return false;  // CRC校验失败
    }
    
    // 解析字段 - 仅提取转速信息
    motor_data->speed_rpm = ((float)((feedback->speed_h << 8) | feedback->speed_l)) * 100.0f / 7.0f;
    motor_data->valid = true;
    motor_data->timestamp = HAL_GetTick();
    
    // 以下字段不需要，注释掉
    // motor_data->temp_c = (float)feedback->temp;
    // motor_data->voltage_v = ((float)((feedback->voltage_h << 8) | feedback->voltage_l)) * 0.01f;
    // motor_data->current_a = ((float)((feedback->current_h << 8) | feedback->current_l)) * 0.01f;
    // motor_data->capacity_mah = (float)((feedback->capacity_h << 8) | feedback->capacity_l);
    
    return true;
}

/**
 * @brief 获取电机数据
 */
bool TelemetryTask_GetMotorData(uint8_t motor_id, MotorData_t *motor_data)
{
    if (motor_id >= 2 || !motor_data) {
        return false;
    }
    
    memcpy(motor_data, &g_MotorData[motor_id], sizeof(MotorData_t));
    return g_MotorData[motor_id].valid;
}

/* ======================== 电机数据处理函数 ======================== */

/**
 * @brief 处理UART3 (电机4) 的自动反馈数据
 */
static void Process_Motor4_Feedback(void)
{
    uint8_t *rx_buffer;
    uint16_t rx_count;
    
    rx_buffer = BSP_UART3_GetRxBuffer();
    rx_count = BSP_UART3_GetRxCount();
    
    // 查找完整的电机反馈包 (10字节)
    if (rx_count >= sizeof(MotorFeedback_t)) {
        // 尝试从缓冲中解析数据
        // 简单方案: 使用最后10字节
        if (TelemetryTask_ParseMotorFeedback(
            &rx_buffer[rx_count - sizeof(MotorFeedback_t)],
            sizeof(MotorFeedback_t),
            &g_MotorData[0])) {
            
            // 解析成功, 清空缓冲准备接收下一包
            BSP_UART3_ClearRxBuffer();
        }
    }
}

/**
 * @brief 处理UART4 (电机3) 的自动反馈数据
 */
static void Process_Motor3_Feedback(void)
{
    uint8_t *rx_buffer;
    uint16_t rx_count;
    
    rx_buffer = BSP_UART4_GetRxBuffer();
    rx_count = BSP_UART4_GetRxCount();
    
    // 查找完整的电机反馈包 (10字节)
    if (rx_count >= sizeof(MotorFeedback_t)) {
        // 尝试从缓冲中解析数据
        // 简单方案: 使用最后10字节
        if (TelemetryTask_ParseMotorFeedback(
            &rx_buffer[rx_count - sizeof(MotorFeedback_t)],
            sizeof(MotorFeedback_t),
            &g_MotorData[1])) {
            
            // 解析成功, 清空缓冲准备接收下一包
            BSP_UART4_ClearRxBuffer();
        }
    }
}

/* ======================== 舵机初始化和查询 ======================== */

/**
 * @brief 初始化舵机 (UART5 单总线)
 * @note  流程:
 *        1. 依次给舵机分配ID号 (从SERVO_BASE_ID开始)
 *        2. 验证舵机在线状态
 */
static uint8_t Initialize_Servos(void)
{
    uint8_t i;
    HAL_StatusTypeDef status;
    uint32_t init_start;
    uint8_t ret=0;

    init_start = HAL_GetTick();
    
    for (i = 0; i < SERVO_COUNT; i++) {
        // 尝试为舵机分配ID
        // 首次初始化时，舵机ID为0 (未编程)
        status = BSP_Servo_SetID(0, SERVO_BASE_ID + i);
        
        if (status == HAL_OK) {
            g_ServoOnline[i] = true;
        } else {
            g_ServoOnline[i] = false;
        }
        
        // 检查初始化超时
        if (HAL_GetTick() - init_start > SERVO_ID_INIT_TIMEOUT_MS) {
            ret |= 1 << i;
					break;
        }
        
        osDelay(10);  // 舵机处理延迟
    }
    return ret;
}

/**
 * @brief 查询所有舵机位置
 */
static void Query_All_Servos(TelemetryData_t *telem)
{
    uint8_t i;
    HAL_StatusTypeDef status;
    float angle;
    
    for (i = 0; i < SERVO_COUNT; i++) {
        if (!g_ServoOnline[i]) {
            continue;  // 跳过离线舵机
        }
        
        // 查询舵机位置
        status = BSP_Servo_QueryAngle(SERVO_BASE_ID + i, &angle);
        
        if (status == HAL_OK) {
            // 更新DataHub
            telem->servo[i].angle_deg = angle;
            telem->servo[i].online = true;
        } else {
            // 标记为离线
            g_ServoOnline[i] = false;
            telem->servo[i].online = false;
        }
        
        osDelay(10);
    }
}

/* ======================== 主任务函数 ======================== */

/**
 * @brief 遥测数据采集任务
 */
void StartTelemetryTask(void *argument)
{
    TelemetryData_t telem;
    uint32_t last_motor_check = 0;
    uint32_t last_servo_query = 0;
    bool servo_initialized = false;
    
    // 初始化所有UART
    BSP_MultiUART_Init();
    osDelay(100);
    
    // 舵机初始化 (一次性)
    uint8_t servo_init_result = Initialize_Servos();
    if (servo_init_result != 0) {
        // 初始化失败处理 (可以记录日志或重试)
        while(Initialize_Servos() != 0) {
            osDelay(1000);  // 每秒重试一次
        }
    } else {
        servo_initialized = true;
    }
    
    last_servo_query = HAL_GetTick();
    
    for(;;) {
				#ifdef SIMULATOR_MODE
					printf("%s\n", __FUNCTION__);
				#endif
        uint32_t now = HAL_GetTick();
        
        // 1. 处理电机反馈数据 (UART3/4)
        if (now - last_motor_check >= MOTOR_PARSE_CHECK_MS) {
            Process_Motor4_Feedback();  // 电机4
            Process_Motor3_Feedback();  // 电机3
            last_motor_check = now;
        }
        
        // 2. 查询舵机位置 (UART5)
        if (now - last_servo_query >= SERVO_QUERY_INTERVAL_MS && servo_initialized) {
            Query_All_Servos(&telem);
            last_servo_query = now;
        }
        
        // 3. 更新DataHub遥测数据
        telem.valid = 1;
        telem.timestamp = now;
        
        // 舵机数据已在Query_All_Servos中更新
        
        // 检查电机数据是否超时
        for (uint8_t i = 0; i < 2; i++) {
            if (g_MotorData[i].valid && (now - g_MotorData[i].timestamp) > MOTOR_DATA_TIMEOUT_MS) {
                g_MotorData[i].valid = false;  // 数据超时，标记为无效
            }
        }
        
        // 填充电调数据
        telem.esc[0].rpm = (uint16_t)g_MotorData[0].speed_rpm;  // 电机4 -> ESC0
        telem.esc[0].online = g_MotorData[0].valid;
        telem.esc[1].rpm = (uint16_t)g_MotorData[1].speed_rpm;  // 电机3 -> ESC1
        telem.esc[1].online = g_MotorData[1].valid;
        // 其他字段暂时设为0或默认值
        telem.esc[0].voltage_mv = 0;
        telem.esc[0].current_ma = 0;
        telem.esc[0].temp_c = 0;
        telem.esc[1].voltage_mv = 0;
        telem.esc[1].current_ma = 0;
        telem.esc[1].temp_c = 0;
        
        DataHub_WriteTelemetry(&telem);
        
        osDelay(10);  // 任务周期
    }
}

/* ======================== 设备在线状态查询 ======================== */

/**
 * @brief 获取设备在线状态
 */
void TelemetryTask_GetOnlineStatus(bool *servo_online, bool *esc_online)
{
    uint8_t i;
    
    if (servo_online) {
        for (i = 0; i < MAX_SERVOS; i++) {
            servo_online[i] = g_ServoOnline[i];
        }
    }
    
    if (esc_online) {
        // ESC在线状态 (如果有的话)
        for (i = 0; i < MAX_ESCS; i++) {
            esc_online[i] = false;  // 暂时设为离线
        }
    }
}
