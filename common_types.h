/**
 ******************************************************************************
 * @file    common_types.h
 * @brief   项目公共数据类型定义 - 集中管理，避免重复定义
 * @note    所有任务应通过包含此文件来使用共用数据结构
 ******************************************************************************
 */

#ifndef __COMMON_TYPES_H
#define __COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======================== 常量定义 ======================== */
#define MAX_SERVOS  3
#define MAX_ESCS    2

/* ======================== 传感器数据结构 ======================== */

/**
 * @brief 传感器数据结构 (IMU + 深度 + 温度)
 */
typedef struct {
    // IMU 数据 (惯性测量单元)
    float roll;              // 横滚角 (度)
    float pitch;             // 俯仰角 (度)
    float yaw;               // 航向角 (度)
    
    // 加速度计
    float accel_x, accel_y, accel_z;  // 加速度 (m/s²)
    
    // 陀螺仪
    float gyro_x, gyro_y, gyro_z;     // 角速度 (度/秒)
    
    // 深度/压力传感器
    float depth_m;           // 深度 (米)
    uint16_t pressure_raw;   // 原始压力值 (ADC)
    
    // 温度传感器
    float temperature;       // 温度 (℃)
    
    // 数据有效标志
    bool valid;              // 数据是否有效
    uint32_t timestamp;      // 时间戳 (ms)
} SensorData_t;

/* ======================== 指令数据结构 ======================== */

/**
 * @brief 命令数据结构 (来自上位机)
 */
typedef struct {
    uint8_t mode;            // 0-待机, 1-手动, 2-自动
    float target_depth;      // 目标深度 (米)
    float target_yaw;        // 目标航向角 (度)
    float target_pitch;      // 目标俯仰角 (度)
    float pitch_cmd;         // 俯仰角指令 (-1.0 ~ 1.0)
    float yaw_cmd;           // 航向角指令 (-1.0 ~ 1.0)
    float roll_cmd;          // 横滚角指令 (-1.0 ~ 1.0)
    float thrust_cmd[2];     // 推进器指令 (-1.0 ~ 1.0)
    uint8_t valve_open;         // 电磁阀状态
    uint16_t seq;            // 序列号
    uint32_t timestamp;      // 时间戳 (ms)
} Command_t;

/**
 * @brief 命令帧格式 (串口协议)
 */
typedef struct __attribute__((packed)) {
    uint8_t header[2];       // 0xBB 0x66
    uint8_t mode;            // 控制模式
    float pitch_cmd;         // 俯仰指令
    float yaw_cmd;           // 航向指令
    float roll_cmd;          // 横滚指令
    float thrust_cmd[2];     // 推进器指令[0]左 [1]右
    float target_depth;      // 目标深度
    float target_pitch;      // 目标俯仰
    float target_yaw;        // 目标航向
    bool valve_open;         // 电磁阀
    uint16_t seq;            // 序列号
    uint16_t checksum;       // CRC校验
    uint8_t tail;            // 0x0A
    uint16_t length;         // 数据长度
} CmdFrame_t;

/* ======================== 执行器反馈数据结构 ======================== */

/**
 * @brief 舵机反馈数据
 */
typedef struct {
    float angle_deg;         // 角度 (度)
    uint16_t current_ma;     // 电流 (mA)
    bool online;             // 是否在线
} ServoTelemetry_t;

/**
 * @brief 电调反馈数据
 */
typedef struct {
    uint16_t rpm;            // 转速 (RPM)
    uint16_t voltage_mv;     // 电压 (mV)
    uint16_t current_ma;     // 电流 (mA)
    uint8_t temp_c;          // 温度 (℃)
    bool online;             // 是否在线
} ESCTelemetry_t;

/**
 * @brief 遥测数据结构
 */
typedef struct {
    ServoTelemetry_t servo[MAX_SERVOS];
    ESCTelemetry_t esc[MAX_ESCS];
    bool valid;              // 数据是否有效
    uint32_t timestamp;      // 时间戳 (ms)
} TelemetryData_t;

/* ======================== 控制输出数据结构 ======================== */

/**
 * @brief 控制输出结构体
 */

typedef struct{
	bool valve_state1;        // 电磁阀状态
	bool valve_state2;
	bool valve_state3;
	bool valve_state4;
	bool valve_state5;
	bool valve_state6;
	bool valve_state7;
	bool valve_state8;
}bState;
typedef union{
		uint8_t val;
		bState bstate;
}valueState;

typedef struct {
    uint8_t ctrl_mode;       // 控制模式
    uint16_t servo_pwm[3];   // 舵机 PWM (500-2500 us)
    uint16_t esc_pwm[2];     // 电调 PWM (1000-2000 us)
		valueState valve_state;
    uint32_t timestamp;      // 时间戳 (ms)
} ControlOutput_t;

/* ======================== 协议通讯数据结构 ======================== */
/**
 * @brief 舵机初始化命令 (UART5 单总线, 图2协议)
 * 命令格式见图2：指定舵机ID号
 */
typedef struct __attribute__((packed)) {
    uint8_t header;            // 包头 (0xFF)
    uint8_t id;                // 当前/待设置ID
    uint8_t cmd;               // 命令字
    uint8_t param;             // 参数 (新ID号或其他参数)
    uint8_t checksum;          // 校验和
} ServoInitCmd_t;


/**
 * @brief 查询舵机状态指令格式
 */
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xAA
    uint8_t device_id;   // 设备 ID: 1-3
    uint8_t cmd;         // 命令: 0x01-查询状态
    uint8_t checksum;    // 校验和
} ServoQueryCmd_t;

/**
 * @brief 舵机响应格式
 */

/**
 * @brief 舵机反馈数据 (UART5 单总线)
 */
typedef struct __attribute__((packed)) {
    uint8_t header;            // 响应包头
    uint8_t id;                // 舵机ID
    uint8_t length;            // 数据长度
    uint8_t angle_h;           // 角度高字节
    uint8_t angle_l;           // 角度低字节
    uint8_t checksum;          // 校验和
} ServoResponse_t;

/**
 * @brief 查询电调状态指令格式
 */
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xCC
    uint8_t device_id;   // 设备 ID: 1-2
    uint8_t cmd;         // 0x02-查询遥测
    uint8_t checksum;
} ESCQueryCmd_t;

/**
 * @brief 电调响应格式
 */
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xDD
    uint8_t device_id;
    uint16_t rpm;        // 转速
    uint16_t voltage;    // 电压 (mV)
    uint16_t current;    // 电流 (mA)
    uint8_t temp;        // 温度 (℃)
    uint8_t checksum;
} ESCResponse_t;

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_TYPES_H */
