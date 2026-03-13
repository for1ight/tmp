/**
 ******************************************************************************
 * @file    imu_driver.c
 * @brief   Yesense IMU 驱动实现 - 协议解析和数据提取
 ******************************************************************************
 */

#include "imu_driver.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

/* ======================== 私有变量 ======================== */
static IMU_Frame_t g_CurrentFrame = {0};
static uint16_t g_FramePos = 0;
static bool g_FrameComplete = false;
static osMutexId_t g_IMUMutex;

/* ======================== 私有函数 ======================== */

/**
 * @brief 从原始字节提取 float (小端格式)
 */
static float ExtractFloat(const uint8_t *data)
{
    union {
        uint8_t bytes[4];
        float value;
    } converter;
    
    converter.bytes[0] = data[0];
    converter.bytes[1] = data[1];
    converter.bytes[2] = data[2];
    converter.bytes[3] = data[3];
    
    return converter.value;
}

/**
 * @brief 从原始字节提取 int32 (小端格式)
 */
static int32_t ExtractInt32(const uint8_t *data)
{
    return (int32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
}

/**
 * @brief 从原始字节提取 uint32 (小端格式)
 */
static uint32_t ExtractUInt32(const uint8_t *data)
{
    return (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
}

/**
 * @brief 从原始字节提取 int16 (小端格式)
 */
static int16_t ExtractInt16(const uint8_t *data)
{
    return (int16_t)((data[0] << 0) | (data[1] << 8));
}

/* ======================== 公共函数 ======================== */

void IMU_Init(void)
{
    // 创建互斥锁
    g_IMUMutex = osMutexNew(NULL);
    
    // 初始化变量
    memset(&g_CurrentFrame, 0, sizeof(IMU_Frame_t));
    g_FramePos = 0;
    g_FrameComplete = false;
}

/**
 * @brief 处理接收到的每个字节
 * @note 实现一个小状态机,用于同步和帧解析
 */
int IMU_ProcessByte(uint8_t byte)
{
    static uint8_t header_count = 0;
    
    // 寻找帧头
    if (g_FramePos == 0) {
        if (byte == IMU_PROTOCOL_HEADER1) {
            g_CurrentFrame.header1 = byte;
            g_FramePos = 1;
            return 0;  // 继续接收
        }
        return 0;
    } else if (g_FramePos == 1) {
        if (byte == IMU_PROTOCOL_HEADER2) {
            g_CurrentFrame.header2 = byte;
            g_FramePos = 2;
            return 0;
        } else {
            // 帧头错误,重新同步
            if (byte == IMU_PROTOCOL_HEADER1) {
                g_CurrentFrame.header1 = byte;
                g_FramePos = 1;
            } else {
                g_FramePos = 0;
            }
            return 0;
        }
    }
    
    // 接收 TID (2 字节)
    if (g_FramePos < 4) {
        uint8_t *tid_ptr = (uint8_t *)&g_CurrentFrame.tid;
        tid_ptr[g_FramePos - 2] = byte;
        g_FramePos++;
        return 0;
    }
    
    // 接收 PAYLOAD LEN (1 字节)
    if (g_FramePos == 4) {
        g_CurrentFrame.payload_len = byte;
        if (byte == 0 || byte > 250) {
            // 长度无效
            g_FramePos = 0;
            return -1;
        }
        g_FramePos++;
        return 0;
    }
    
    // 接收 PAYLOAD (可变长度)
    if (g_FramePos < (5 + g_CurrentFrame.payload_len)) {
        uint16_t payload_idx = g_FramePos - 5;
        g_CurrentFrame.payload[payload_idx] = byte;
        g_FramePos++;
        return 0;
    }
    
    // 接收校验和 CK1
    if (g_FramePos == (5 + g_CurrentFrame.payload_len)) {
        g_CurrentFrame.ck1 = byte;
        g_FramePos++;
        return 0;
    }
    
    // 接收校验和 CK2
    if (g_FramePos == (6 + g_CurrentFrame.payload_len)) {
        g_CurrentFrame.ck2 = byte;
        g_FramePos++;
        
        // 帧接收完整,进行校验
        if (IMU_VerifyChecksum(&g_CurrentFrame)) {
            g_FrameComplete = true;
            g_FramePos = 0;
            return 1;  // 数据帧完整且有效
        } else {
            // 校验失败,重新同步
            g_FramePos = 0;
            return -1;
        }
    }
    
    return 0;
}

/**
 * @brief 计算校验和 (Fletcher-16)
 * @note CRC 从 TID 开始,到 payload 最后一个字节结束
 */
void IMU_CalcChecksum(const uint8_t *data, uint16_t len, uint8_t *ck1, uint8_t *ck2)
{
    uint8_t sum1, sum2;
    uint16_t i;
    
    if (data == NULL || ck1 == NULL || ck2 == NULL) return;
    
    sum1 = 0;
    sum2 = 0;
    
    for (i = 0; i < len; i++) {
        sum1 += data[i];
        sum2 += sum1;
    }
    
    *ck1 = sum1;
    *ck2 = sum2;
}

/**
 * @brief 验证帧的校验和
 */
bool IMU_VerifyChecksum(const IMU_Frame_t *frame)
{
    uint16_t crc_len;
    const uint8_t *crc_data;
    uint8_t ck1_calc, ck2_calc;
    
    if (frame == NULL) return false;
    
    // CRC 计算范围: TID + LEN + PAYLOAD
    crc_len = 2 + 1 + frame->payload_len;  // TID(2B) + LEN(1B) + PAYLOAD(N)
    crc_data = (const uint8_t *)&frame->tid;
    
    IMU_CalcChecksum(crc_data, crc_len, &ck1_calc, &ck2_calc);
    
    return (frame->ck1 == ck1_calc) && (frame->ck2 == ck2_calc);
}

/**
 * @brief 解析协议帧中的各项数据
 */
bool IMU_ParseFrame(const IMU_Frame_t *frame, IMU_Data_t *output)
{
    uint16_t pos;
    const uint8_t *payload;
    uint8_t data_id;
		// 解析 payload
    uint8_t data_len;
    uint8_t *data_ptr;
    pos = 0;
    
    if (frame == NULL || output == NULL) return false;
    
    // 检查帧头
    if (frame->header1 != IMU_PROTOCOL_HEADER1 || frame->header2 != IMU_PROTOCOL_HEADER2) {
        return false;
    }
    
    // 验证校验和
    if (!IMU_VerifyChecksum(frame)) {
        return false;
    }
    
    // 初始化输出数据
    memset(output, 0, sizeof(IMU_Data_t));
    output->valid = true;
    output->last_update = HAL_GetTick();
    

    payload = frame->payload;
    
    while (pos < frame->payload_len) {
        data_id = payload[pos];
        data_len = payload[pos + 1];
        data_ptr = (uint8_t *)&payload[pos + 2];
        
        switch (data_id) {
            // 加速度计 (3 x float = 12 bytes)
            case IMU_ACCEL_ID:
                if (data_len >= 12) {
                    output->accel.x = ExtractFloat(&data_ptr[0]);
                    output->accel.y = ExtractFloat(&data_ptr[4]);
                    output->accel.z = ExtractFloat(&data_ptr[8]);
                }
                break;
            
            // 陀螺仪/角速度 (3 x float = 12 bytes)
            case IMU_ANGLE_RATE_ID:
                if (data_len >= 12) {
                    output->gyro.x = ExtractFloat(&data_ptr[0]);
                    output->gyro.y = ExtractFloat(&data_ptr[4]);
                    output->gyro.z = ExtractFloat(&data_ptr[8]);
                }
                break;
            
            // 磁力计 (3 x float = 12 bytes)
            case IMU_MAGNETIC_ID:
                if (data_len >= 12) {
                    output->mag.x = ExtractFloat(&data_ptr[0]);
                    output->mag.y = ExtractFloat(&data_ptr[4]);
                    output->mag.z = ExtractFloat(&data_ptr[8]);
                }
                break;
            
            // 欧拉角 (3 x float = 12 bytes)
            case IMU_EULER_ID:
                if (data_len >= 12) {
                    output->attitude.pitch = ExtractFloat(&data_ptr[0]);
                    output->attitude.roll = ExtractFloat(&data_ptr[4]);
                    output->attitude.yaw = ExtractFloat(&data_ptr[8]);
                }
                break;
            
            // 四元数 (4 x float = 16 bytes)
            case IMU_QUATERNION_ID:
                if (data_len >= 16) {
                    output->quaternion.q0 = ExtractFloat(&data_ptr[0]);
                    output->quaternion.q1 = ExtractFloat(&data_ptr[4]);
                    output->quaternion.q2 = ExtractFloat(&data_ptr[8]);
                    output->quaternion.q3 = ExtractFloat(&data_ptr[12]);
                }
                break;
            
            // 传感器温度 (1 x int16_t = 2 bytes, 单位 0.01°C)
            case IMU_SENSOR_TEMP_ID:
                if (data_len >= 2) {
                    int16_t temp_raw = ExtractInt16(data_ptr);
                    output->temperature = temp_raw * IMU_SENSOR_TEMP_FACTOR;
                }
                break;
            
            // 采样时间戳 (1 x uint32_t = 4 bytes, 微秒)
            case IMU_TIMESTAMP_ID:
                if (data_len >= 4) {
                    output->sample_timestamp = ExtractUInt32(data_ptr);
                }
                break;
            
            // 数据就绪时间戳 (1 x uint32_t = 4 bytes)
            case IMU_DATA_READY_TIMESTAMP_ID:
                if (data_len >= 4) {
                    output->data_ready_timestamp = ExtractUInt32(data_ptr);
                }
                break;
            
            default:
                // 未知数据 ID,跳过
                break;
        }
        
        // 移动到下一个数据段
        pos += 2 + data_len;
    }
    
    return true;
}

/**
 * @brief 获取最新 IMU 数据
 */
bool IMU_GetData(IMU_Data_t *data)
{
    if (data == NULL) return false;
    
    // 检查是否有新数据
    if (!g_FrameComplete) {
        return false;
    }
    
    osMutexAcquire(g_IMUMutex, osWaitForever);
    
    // 解析帧
    if (IMU_ParseFrame(&g_CurrentFrame, data)) {
        g_FrameComplete = false;
        
        osMutexRelease(g_IMUMutex);
        return true;
    }
    
    osMutexRelease(g_IMUMutex);
    return false;
}

/**
 * @brief USART2 接收完成中断回调 (需在 stm32f4xx_it.c 中调用)
 */
void IMU_USART_RxCallback(uint8_t byte)
{
    // 处理接收到的字节
    int result = IMU_ProcessByte(byte);
    
    if (result == 1) {
        // 帧接收完整
        // 可选: 触发事件或发送信号给 task_sensor
    } else if (result == -1) {
        // 帧接收出错
        // 可选: 错误计数或日志
    }
}
