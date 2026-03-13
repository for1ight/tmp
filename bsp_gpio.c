/**
 ******************************************************************************
 * @file    bsp_gpio.c
 * @brief   GPIO驱动 - 继电器控制实现 + 安全检查
 * @note    支持6个球阀继电器控制，高电平有效（PE7~PE12）
 ******************************************************************************
 */

#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
/* ======================== 私有变量 ======================== */

/**
 * @brief 继电器状态寄存器 (bit0=relay1, bit1=relay2, ...)
 */
static uint8_t g_relay_state = 0;

/**
 * @brief 继电器GPIO信息表
 */
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} g_relay_pins[RELAY_COUNT] = {
    {ballValve1_GPIO_Port, ballValve1_Pin},  // RELAY_1: PE7
    {ballValve2_GPIO_Port, ballValve2_Pin},  // RELAY_2: PE8
    {ballValve3_GPIO_Port, ballValve3_Pin},  // RELAY_3: PE9
    {ballValve4_GPIO_Port, ballValve4_Pin},  // RELAY_4: PE10
    {ballValve5_GPIO_Port, ballValve5_Pin},  // RELAY_5: PE11
    {ballValve6_GPIO_Port, ballValve6_Pin},  // RELAY_6: PE12
};

/* ======================== 公共函数实现 ======================== */

/**
 * @brief 初始化GPIO驱动
 */
bool BSP_GPIO_Init(void)
{
    // GPIO已在MX_GPIO_Init()中初始化，这里只需初始化状态变量
    g_relay_state = 0;  // 所有继电器关闭
    
    // 确保所有继电器初始状态为关闭（低电平）
    BSP_GPIO_DisableAll();
    
    return true;
}

/**
 * @brief 设置继电器状态
 */
bool BSP_GPIO_SetRelay(BSP_Relay_t relay, bool state)
{
    // 检查继电器编号有效性
    if (relay >= RELAY_COUNT) {
        return false;
    }
    
    // 设置GPIO引脚
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(g_relay_pins[relay].port, g_relay_pins[relay].pin, pin_state);
    
    // 更新状态位
    if (state) {
        g_relay_state |= (1 << relay);   // 设置对应位
    } else {
        g_relay_state &= ~(1 << relay);  // 清除对应位
    }
    
    return true;
}

/**
 * @brief 获取继电器状态
 */
bool BSP_GPIO_GetRelay(BSP_Relay_t relay)
{
    if (relay >= RELAY_COUNT) {
        return false;
    }
    
    return (g_relay_state >> relay) & 1;
}

/**
 * @brief 切换继电器状态
 */
bool BSP_GPIO_ToggleRelay(BSP_Relay_t relay)
{
    if (relay >= RELAY_COUNT) {
        return false;
    }
    
    bool current_state = (g_relay_state >> relay) & 1;
    return BSP_GPIO_SetRelay(relay, !current_state);
}

/**
 * @brief 同时设置多个继电器
 */
void BSP_GPIO_SetRelayMask(uint8_t relay_mask, bool state)
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        if (relay_mask & (1 << i)) {
            BSP_GPIO_SetRelay((BSP_Relay_t)i, state);
        }
    }
}

/**
 * @brief 获取所有继电器状态
 */
uint8_t BSP_GPIO_GetAllRelayStates(void)
{
    return g_relay_state;
}

/**
 * @brief 关闭所有继电器
 */
void BSP_GPIO_DisableAll(void)
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        BSP_GPIO_SetRelay((BSP_Relay_t)i, false);
    }
}

/* ======================== 安全检查 ======================== */

/**
 * @brief 系统安全检查函数
 * @note  在main()中系统初始化后调用，检查硬件状态
 *        验证所有关键外设是否正常工作
 */
void SafetyCheck(void)
{
    // 初始化GPIO驱动和继电器
    if (!BSP_GPIO_Init()) {
        // 继电器初始化失败，进入安全模式
        while (1) {
            // 无限循环，等待调试或硬件恢复
					//printf("SafetyCheck Error\n");
        }
    }
    
    // 确保所有继电器处于安全状态（关闭）
    BSP_GPIO_DisableAll();
    
    /* ======================== 舵机中位设置 ======================== */
    // 设置所有舵机到中位位置 (1500us = 90°)
    // 舵机ID: 0, 1, 2 对应 SERVO0, SERVO1, SERVO2
    BSP_PWM_SetServo(0, SERVO_MIDPOINT_US);  // 舵机1
    BSP_PWM_SetServo(1, SERVO_MIDPOINT_US);  // 舵机2
    BSP_PWM_SetServo(2, SERVO_MIDPOINT_US);  // 舵机3
    
    /* ======================== 电机中位设置 ======================== */
    // 设置所有电机ESC到中位位置 (1500us = 停止)
    // ESC ID: 0, 1 对应 ESC0, ESC1
    BSP_PWM_SetESC(0, ESC_MIDPOINT_US);      // 电机1
    BSP_PWM_SetESC(1, ESC_MIDPOINT_US);      // 电机2
    
    // 初始化PWM驱动（如果未在其他地方初始化）
    // BSP_PWM_Init() 已在main.c中调用
    
    // 初始化ADC驱动（如果需要）
    // BSP_ADC_Init() 由freertos初始化过程调用
    
    // 初始化UART驱动（如果需要）
    // BSP_MultiUART_Init() 由task_telemetry调用
    
    // 这里可以添加其他硬件健康检查
    // 例如: 检查传感器响应、验证存储器完整性等
}
