/**
 ******************************************************************************
 * @file    task_sensor.c
 * @brief   传感器采集任务 - 采集 IMU、压力、温度等数据
 * @note    频率 50Hz, 从 ADC 和 USART2 (IMU) 读取数据
 ******************************************************************************
 */

#include "task_sensor.h"
#include "data_hub.h"
#include "imu_driver.h"
#include "adc.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_adc.h"
#include <string.h>
#include <math.h>

/* ======================== 配置参数 ======================== */
#define SENSOR_TASK_FREQ_HZ     50      // 采集频率 50Hz
#define SENSOR_TASK_PERIOD_MS   (1000 / SENSOR_TASK_FREQ_HZ)

/* ======================== 私有变量 ======================== */
static SensorData_t g_SensorData = {0};
static IMU_Data_t g_IMUData = {0};
static uint8_t g_USART2_RxBuf[256];
static volatile uint16_t g_USART2_RxLen = 0;
static ADC_ProcessedData_t g_ADC_Data = {0};
static bool g_ADC_DataValid = false;
static uint32_t g_ADC_SampleCount = 0;
static uint32_t g_ADC_ErrorCount = 0;

/* ======================== 私有函数 ======================== */

/**
 * @brief 读取 ADC 压力/深度数据 (使用DMA)
 */
static void ReadPressureADC(void)
{
    // 从ADC DMA驱动获取处理后的数据
    if (BSP_ADC_GetProcessedData(&g_ADC_Data)) {
        g_ADC_DataValid = true;
        g_ADC_SampleCount++;
        
        // 将原始压力值保存
        ADC_RawData_t raw_data;
        if (BSP_ADC_GetRawData(&raw_data)) {
            g_SensorData.pressure_raw = raw_data.pressure_raw;
        }
        
        // 使用压力值计算深度 (水中: depth = (pressure - P_atm) / (rho * g))
        // P_atm = 101325 Pa, rho = 1025 kg/m^3 (海水), g = 9.81 m/s^2
        const float P_ATMOSPHERIC = 101325.0f;
        const float RHO_SEAWATER = 1025.0f;
        const float G = 9.81f;
        
        g_SensorData.depth_m = (g_ADC_Data.pressure_pa - P_ATMOSPHERIC) / (RHO_SEAWATER * G);
        if (g_SensorData.depth_m < 0.0f) {
            g_SensorData.depth_m = 0.0f;  // 限制最小深度
        }
        
        // 更新温度数据 (可来自ADC或IMU)
        if (g_ADC_Data.temp_board > -100.0f && g_ADC_Data.temp_board < 100.0f) {
            // 如果ADC温度有效,优先使用ADC温度
            g_SensorData.temperature = g_ADC_Data.temp_board;
            // 否则使用IMU提供的温度
        }
    } else {
        g_ADC_ErrorCount++;
        g_ADC_DataValid = false;
    }
}


/**
 * @brief 从 IMU 读取数据
 */
static void ReadIMUData(void)
{
    // 获取 IMU 数据 (从 imu_driver 中读取最新解析的数据)
    if (IMU_GetData(&g_IMUData)) {
        // 更新传感器数据结构
        g_SensorData.roll = g_IMUData.attitude.roll;
        g_SensorData.pitch = g_IMUData.attitude.pitch;
        g_SensorData.yaw = g_IMUData.attitude.yaw;
        // 无运动控制 不需要加速度和陀螺仪数据
        // g_SensorData.accel_x = g_IMUData.accel.x;
        // g_SensorData.accel_y = g_IMUData.accel.y;
        // g_SensorData.accel_z = g_IMUData.accel.z;
        
        // g_SensorData.gyro_x = g_IMUData.gyro.x;
        // g_SensorData.gyro_y = g_IMUData.gyro.y;
        // g_SensorData.gyro_z = g_IMUData.gyro.z;
        
        // g_SensorData.temperature = g_IMUData.temperature;
    }
}

/**
 * @brief 简单的姿态融合 (可选)
 * @note 当 IMU 提供的欧拉角不够准确时,可进行额外融合
 */
static void FuseAttitude(void)
{
    // IMU 已经提供欧拉角,这里可以添加卡尔曼滤波等算法
    // 为简化起见,直接使用 IMU 输出
    // (实际项目可在此添加更复杂的融合算法)
}

/* ======================== 任务入口 ======================== */
static uint32_t count=0;
void StartSensorTask(void *argument)
{
    // 初始化传感器和驱动
    memset(&g_SensorData, 0, sizeof(SensorData_t));
    g_SensorData.valid = false;
    count++;
    // 初始化 ADC DMA 驱动
    if (!BSP_ADC_Init()) {
        // ADC初始化失败,进入错误循环
        while (1) {
            osDelay(1000);
        }
    }
    
    // 启动 ADC DMA 采样
    if (!BSP_ADC_StartDMA()) {
        // ADC DMA启动失败
        while (1) {
            osDelay(1000);
        }
    }
    
    // 初始化 IMU 驱动
    IMU_Init();
    
    // 启动 USART2 DMA 接收 (用于接收 IMU 数据)
    uint32_t last_wake_time;
    uint32_t start_time;
    uint32_t elapsed;
    
    last_wake_time = osKernelGetTickCount();
    
    HAL_UART_Receive_DMA(&huart2, g_USART2_RxBuf, sizeof(g_USART2_RxBuf));
    
    for(;;)
    {
				#ifdef SIMULATOR_MODE
					printf("%s\n", __FUNCTION__);
				#endif
        start_time = HAL_GetTick();
        
        // ========== 读取所有传感器 ==========
        ReadPressureADC();      // 深度/压力传感器 (ADC DMA)
        ReadIMUData();          // IMU 数据 (通过 USART2)
        
        // ========== 数据处理 ==========
        //FuseAttitude();         // 姿态融合 (可选)
        
        // ========== 更新时间戳 ==========
        g_SensorData.timestamp = HAL_GetTick();
        g_SensorData.valid = g_ADC_DataValid;  // 仅当ADC数据有效时标记数据有效
        
        // ========== 写入数据中心 ==========
        DataHub_WriteSensor(&g_SensorData);
        
        // ========== 任务同步 ==========
        // 维持 50Hz 周期
        elapsed = HAL_GetTick() - start_time;
        if (elapsed < SENSOR_TASK_PERIOD_MS) {
            osDelay(SENSOR_TASK_PERIOD_MS - elapsed);
        }
    }
}

bool SensorTask_GetData(SensorData_t *data)
{
    if (data == NULL) return false;
    
    if (DataHub_ReadSensor(data)) {
        return data->valid;
    }
    return false;
}

/* ======================== 调试接口实现 ======================== */

bool SensorTask_GetADCStats(uint32_t *sample_count, uint32_t *error_count)
{
    if (!sample_count || !error_count) {
        return false;
    }
    
    *sample_count = g_ADC_SampleCount;
    *error_count = g_ADC_ErrorCount;
    
    return true;
}

bool SensorTask_GetLastADCData(float *pressure_pa, float *depth_m, float *temperature)
{
    if (!pressure_pa || !depth_m || !temperature) {
        return false;
    }
    
    if (!g_ADC_DataValid) {
        return false;
    }
    
    *pressure_pa = g_ADC_Data.pressure_pa;
    *depth_m = g_SensorData.depth_m;
    *temperature = g_ADC_Data.temp_board;
    
    return true;
}

void SensorTask_ResetADCStats(void)
{
    g_ADC_SampleCount = 0;
    g_ADC_ErrorCount = 0;
}

/**
 * @brief USART2 半缓冲区完成回调
 * @note 用于环形缓冲处理
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t i;
    
    if (huart->Instance == USART2) {
        // 处理前半部分缓冲区
        for (i = 0; i < sizeof(g_USART2_RxBuf) / 2; i++) {
            IMU_ProcessByte(g_USART2_RxBuf[i]);
        }
    }
}

/**
 * @brief UART DMA 接收完成回调 (处理所有UART)
 * @note 此函数由HAL库在DMA接收完成时调用
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t i;
    
    // USART2: IMU数据 (在task_sensor中处理)
    if (huart->Instance == USART2) {
        // 处理后半部分缓冲区
        for (i = sizeof(g_USART2_RxBuf) / 2; i < sizeof(g_USART2_RxBuf); i++) {
            IMU_ProcessByte(g_USART2_RxBuf[i]);
        }
        // 重新启动DMA接收以实现环形缓冲
        HAL_UART_Receive_DMA(&huart2, g_USART2_RxBuf, sizeof(g_USART2_RxBuf));
    }
    // UART5: 舵机数据 (由task_telemetry处理)
    else if (huart->Instance == UART5) {
        // 释放接收完成信号量给task_telemetry
        if (g_UART5_RxCompleteSem != NULL) {
            osSemaphoreRelease(g_UART5_RxCompleteSem);
        }
    }
    // UART3, UART4: 电机数据 (由task_telemetry处理)
    else if (huart->Instance == USART3) {
        g_UART3_RxCount = 256;  // 标记UART3接收完成
    }
    else if (huart->Instance == UART4) {
        g_UART4_RxCount = 256;  // 标记UART4接收完成
    }
}

/**
 * @brief 备选: 如果使用 USART 空闲中断
 * @note 这提供了另一种接收方式 (不定长数据)
 */
void USART2_IDLE_Callback(void)
{
    // 可选: 实现空闲中断处理
    // 这在 IMU 以不定长数据帧发送时很有用
}

