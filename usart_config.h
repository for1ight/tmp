/**
 ******************************************************************************
 * @file    usart_config.h
 * @brief   USART 配置说明 - 串口配置参考
 ******************************************************************************
 * 
 * 本项目的串口配置如下:
 * 
 * 1. USART1 (PA9/PA10)
 *    - 用途: 与上位机通讯
 *    - 波特率: 115200 bps
 *    - 数据位: 8 bit
 *    - 停止位: 1 bit
 *    - 奇偶校验: 无
 *    - 流控: 无
 *    - 配置: TTL 串口
 *    - 任务: StartUplinkTask (发送遥测数据)
 *           StartCmdTask (接收命令)
 * 
 * 2. USART2 (PA2/PA3)
 *    - 用途: IMU (Yesense) 通讯
 *    - 波特率: 115200 bps (根据 IMU 实际设置调整)
 *    - 数据位: 8 bit
 *    - 停止位: 1 bit
 *    - 奇偶校验: 无
 *    - 流控: 无
 *    - 配置: DMA 接收模式
 *    - 协议: Yesense 标准协议
 *           帧格式: 0x59 0x53 + TID(2B) + LEN(1B) + PAYLOAD + CK1 + CK2
 *    - 任务: StartSensorTask (接收 IMU 数据并解析)
 * 
 * 3. USART3 (可选，暂未使用)
 *    - 用途: 备用调试接口或其他传感器
 * 
 * ====== CubeMX 配置步骤 ======
 * 
 * USART1 配置:
 * - Pinout: PA9 (TX), PA10 (RX)
 * - Mode: Asynchronous
 * - Baudrate: 115200
 * - DMA Settings: 
 *   - TX DMA: USART1_TX on DMA2 Stream 7 (Memory to Peripheral)
 *   - RX: 普通中断模式或 DMA
 * - NVIC: USART1 global interrupt 启用
 * 
 * USART2 配置:
 * - Pinout: PA2 (TX), PA3 (RX)
 * - Mode: Asynchronous
 * - Baudrate: 115200
 * - DMA Settings:
 *   - RX DMA: USART2_RX on DMA1 Stream 5 (Peripheral to Memory, Circular)
 *   - TX: 普通发送 (可选 DMA)
 * - NVIC: USART2 global interrupt 启用
 * 
 * ====== 代码集成说明 ======
 * 
 * USART2 接收中断处理 (在 stm32f4xx_it.c 中):
 * 
 *   void USART2_IRQHandler(void)
 *   {
 *       HAL_UART_IRQHandler(&huart2);
 *   }
 * 
 * USART2 接收完成回调 (已在 task_sensor.c 中实现):
 * 
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *   {
 *       if (huart->Instance == USART2) {
 *           // 处理 IMU 数据
 *           // 触发 imu_driver 的字节处理
 *       }
 *   }
 * 
 * 如果使用 DMA 接收,需要在 task_sensor.c 中实现:
 * - HAL_UART_RxCpltCallback(): 处理半缓冲或完整缓冲
 * - HAL_UART_RxHalfCpltCallback(): 处理半缓冲完成
 * - 将接收到的字节逐个传递给 IMU_ProcessByte()
 * 
 * ====== IMU 协议特性 ======
 * 
 * Yesense IMU 使用标准协议格式:
 * - 帧头: 0x59 0x53 (固定)
 * - TID: 2 字节事务 ID
 * - LEN: 1 字节负载长度
 * - PAYLOAD: 可变长度数据,包含多种数据类型 (见 imu_driver.h)
 * - CK1, CK2: Fletcher-16 校验和
 * 
 * 数据类型包括:
 * - 0x01: 温度 (2B)
 * - 0x10: 加速度计 (12B - 3 x float)
 * - 0x20: 角速度 (12B - 3 x float)
 * - 0x30: 磁力计 (12B)
 * - 0x40: 欧拉角 (12B - pitch, roll, yaw)
 * - 0x41: 四元数 (16B)
 * - 0x50: UTC 时间 (11B)
 * - 0x51: 采样时间戳 (4B)
 * - 0x70: 速度 (12B)
 * 
 * ====== 故障排查 ======
 * 
 * 如果 IMU 数据未能正确接收:
 * 
 * 1. 检查硬件连接:
 *    - 确认 USART2 TX/RX 接线正确
 *    - 检查 GND 连接
 *    - 验证电源供应
 * 
 * 2. 验证波特率:
 *    - IMU 默认波特率 (通常 115200)
 *    - 与 CubeMX 配置一致
 * 
 * 3. 数据调试:
 *    - 在 UplinkTask 中添加 IMU 原始数据上传
 *    - 在上位机观察接收到的原始数据
 *    - 验证帧头 (0x59 0x53) 出现位置
 * 
 * 4. DMA 配置:
 *    - 验证 DMA 通道未与其他外设冲突
 *    - 确认环形缓冲区配置正确
 * 
 * ====== 性能建议 ======
 * 
 * - Sensor Task (50Hz): 处理所有传感器数据,优先级 High
 * - Ctrl Task (100Hz): 控制输出,优先级 Realtime (最高)
 * - Telemetry Task (10Hz): 查询电调舵机,优先级 Normal
 * - Cmd Task (20Hz): 接收上位机命令,优先级 AboveNormal
 * - Uplink Task (20Hz): 上传数据,优先级 AboveNormal
 * 
 * 总体 RTOS 堆栈大小需要根据实际情况调整 (见 FreeRTOSConfig.h)
 * 
 ******************************************************************************
 */

#ifndef __USART_CONFIG_H
#define __USART_CONFIG_H

/* USART 波特率配置 */
#define USART1_BAUDRATE    115200  // 上位机通讯
#define USART2_BAUDRATE    115200  // IMU 通讯

#endif /* __USART_CONFIG_H */
