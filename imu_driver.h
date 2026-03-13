/**
 ******************************************************************************
 * @file    imu_driver.h
 * @brief   Yesense IMU 驱动头文件 - 支持标准协议解析
 * @note    协议格式: 0x59 0x53 + TID(2B) + LEN(1B) + PAYLOAD + CK1 + CK2
 ******************************************************************************
 */

#ifndef __IMU_DRIVER_H
#define __IMU_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======================== 协议常量定义 ======================== */
#define IMU_PROTOCOL_HEADER1        0x59
#define IMU_PROTOCOL_HEADER2        0x53
#define IMU_PROTOCOL_MIN_LEN        7   // 最小协议长度

/* ======================== 数据 ID 定义 ======================== */
#define IMU_SENSOR_TEMP_ID          0x01    // 传感器温度
#define IMU_ACCEL_ID                0x10    // 加速度计
#define IMU_ANGLE_RATE_ID           0x20    // 陀螺仪 (角速度)
#define IMU_MAGNETIC_ID             0x30    // 磁场强度 (归一化)
#define IMU_EULER_ID                0x40    // 欧拉角
#define IMU_QUATERNION_ID           0x41    // 四元数
#define IMU_UTC_ID                  0x50    // UTC 时间
#define IMU_TIMESTAMP_ID            0x51    // 采样时间戳
#define IMU_DATA_READY_TIMESTAMP_ID 0x52    // 数据就绪时间戳
#define IMU_LOCATION_ID             0x60    // 位置 (普通精度)
#define IMU_SPEED_ID                0x70    // 速度

/* ======================== 数据长度定义 ======================== */
#define IMU_ACCEL_LEN               12      // 3 x float (4B)
#define IMU_ANGLE_RATE_LEN          12      // 3 x float
#define IMU_EULER_LEN               12      // 3 x float (pitch, roll, yaw)
#define IMU_QUATERNION_LEN          16      // 4 x float
#define IMU_SENSOR_TEMP_LEN         2       // 1 x int16_t

/* ======================== 转换因子 ======================== */
#define IMU_SENSOR_TEMP_FACTOR      0.01f   // 温度单位: 0.01°C
#define IMU_DATA_FACTOR             0.000001f // 一般数据转换因子

/* ======================== 数据结构 ======================== */

/**
 * @brief 3轴数据结构
 */
typedef struct {
    float x;
    float y;
    float z;
} IMU_Axis_t;

/**
 * @brief 欧拉角数据
 */
typedef struct {
    float pitch;        // 俯仰角 (度)
    float roll;         // 横滚角 (度)
    float yaw;          // 航向角 (度)
} IMU_Attitude_t;

/**
 * @brief 四元数数据
 */
typedef struct {
    float q0;           // w 分量
    float q1;           // x 分量
    float q2;           // y 分量
    float q3;           // z 分量
} IMU_Quaternion_t;

/**
 * @brief 完整 IMU 输出数据结构
 */
typedef struct {
    // 加速度计 (m/s²)
    IMU_Axis_t accel;
    
    // 陀螺仪/角速度 (°/s)
    IMU_Axis_t gyro;
    
    // 磁力计 (归一化值)
    IMU_Axis_t mag;
    
    // 欧拉角 (°)
    IMU_Attitude_t attitude;
    
    // 四元数
    IMU_Quaternion_t quaternion;
    
    // 温度 (°C)
    float temperature;
    
    // 时间戳 (微秒)
    uint32_t sample_timestamp;
    uint32_t data_ready_timestamp;
    
    // 数据有效标志
    bool valid;
    uint32_t last_update;
} IMU_Data_t;

/**
 * @brief IMU 协议帧格式
 */
typedef struct __attribute__((packed)) {
    uint8_t header1;        // 0x59
    uint8_t header2;        // 0x53
    uint16_t tid;           // 事务 ID
    uint8_t payload_len;    // 负载长度
    uint8_t payload[250];   // 负载数据
    uint8_t ck1;            // 校验和 1
    uint8_t ck2;            // 校验和 2
} IMU_Frame_t;

/* ======================== 公共函数 ======================== */

/**
 * @brief 初始化 IMU 驱动
 */
void IMU_Init(void);

/**
 * @brief 处理接收到的数据字节
 * @param byte 接收到的单个字节
 * @return 0-继续接收, 1-数据帧完整, -1-出错
 */
int IMU_ProcessByte(uint8_t byte);

/**
 * @brief 解析 IMU 协议数据
 * @param frame 协议帧指针
 * @param output 输出数据结构
 * @return true-成功, false-失败
 */
bool IMU_ParseFrame(const IMU_Frame_t *frame, IMU_Data_t *output);

/**
 * @brief 获取最新 IMU 数据
 * @param data 输出数据指针
 * @return true-成功, false-无新数据
 */
bool IMU_GetData(IMU_Data_t *data);

/**
 * @brief 计算校验和 (Fletcher-16 算法)
 */
void IMU_CalcChecksum(const uint8_t *data, uint16_t len, uint8_t *ck1, uint8_t *ck2);

/**
 * @brief 验证校验和
 */
bool IMU_VerifyChecksum(const IMU_Frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DRIVER_H */
