/**
 ******************************************************************************
 * @file    bsp_pwm.c
 * @brief   PWM driver - servo and ESC control
 * @note    Wraps STM32 HAL timer API, provides high-level API
 ******************************************************************************
 */

#include "bsp_pwm.h"
#include "tim.h"

/* ======================== Configuration ======================== */
#define SERVO_PWM_MIN       500     // Minimum pulse (0.5ms)
#define SERVO_PWM_MAX       2500    // Maximum pulse (2.5ms)
#define SERVO_PWM_CENTER    1500    // Center position (1.5ms)

#define ESC_PWM_MIN         1000    // Minimum pulse
#define ESC_PWM_MAX         2000    // Maximum pulse

/* ======================== Private variables ======================== */
static TIM_HandleTypeDef *g_ServoTimer = &htim2;  // Servo uses TIM2
static TIM_HandleTypeDef *g_ESCTimer = &htim3;    // ESC0 uses TIM3 (shared)

// Channel mapping (based on hardware connections)
static const uint32_t g_ServoChannels[3] = {
    TIM_CHANNEL_1,  // Servo 0 -> TIM2 CH1
    TIM_CHANNEL_3,  // Servo 1 -> TIM2 CH3
    TIM_CHANNEL_4   // Servo 2 -> TIM2 CH4
};

static const uint32_t g_ESCChannels[2] = {
    TIM_CHANNEL_1,  // ESC 0 -> TIM3 CH1
    TIM_CHANNEL_2   // ESC 1 -> TIM3 CH2
};

/* ======================== API implementation ======================== */

void BSP_PWM_Init(void)
{
    // Start PWM output on all servo channels
    HAL_TIM_PWM_Start(g_ServoTimer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_ServoTimer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(g_ServoTimer, TIM_CHANNEL_4);
    
    // Start PWM output on ESC1 (TIM3 CH1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    // Initialize to center/stop positions
    BSP_PWM_SetServo(0, SERVO_PWM_CENTER);
    BSP_PWM_SetServo(1, SERVO_PWM_CENTER);
    BSP_PWM_SetServo(2, SERVO_PWM_CENTER);
    BSP_PWM_SetESC(0, ESC_PWM_MIN);
    BSP_PWM_SetESC(1, ESC_PWM_MIN);
}

void BSP_PWM_SetServo(uint8_t servo_id, uint16_t pulse_us)
{
    if (servo_id >= 3) return;
    
    // Range limit
    if (pulse_us < SERVO_PWM_MIN) pulse_us = SERVO_PWM_MIN;
    if (pulse_us > SERVO_PWM_MAX) pulse_us = SERVO_PWM_MAX;
    
    // Set CCR value using channel mapping array
    __HAL_TIM_SET_COMPARE(g_ServoTimer, g_ServoChannels[servo_id], pulse_us);
}

void BSP_PWM_SetESC(uint8_t esc_id, uint16_t pulse_us)
{
    if (esc_id >= 2) return;
    
    // Range limit
    if (pulse_us < ESC_PWM_MIN) pulse_us = ESC_PWM_MIN;
    if (pulse_us > ESC_PWM_MAX) pulse_us = ESC_PWM_MAX;
    
    // Set CCR value based on ESC ID with correct timer/channel
    __HAL_TIM_SET_COMPARE(g_ESCTimer, g_ESCChannels[esc_id], pulse_us);
}

void BSP_PWM_StopAll(void)
{
    uint8_t i;
    
    // Set all servos to center
    for (i = 0; i < 3; i++) {
        BSP_PWM_SetServo(i, SERVO_PWM_CENTER);
    }
    
    // Set all ESCs to stop
    for (i = 0; i < 2; i++) {
        BSP_PWM_SetESC(i, ESC_PWM_MIN);
    }
}

/**
 * @brief ESC calibration mode (advanced, do not use normally)
 * @note Sequence: Maximum pulse -> Minimum pulse -> Ready
 */
void BSP_PWM_ESC_Calibrate(void)
{
    // 1. Send maximum pulse (2000us)
    BSP_PWM_SetESC(0, ESC_PWM_MAX);
    BSP_PWM_SetESC(1, ESC_PWM_MAX);
    HAL_Delay(3000);  // Wait for confirmation beep
    
    // 2. Send minimum pulse (1000us)
    BSP_PWM_SetESC(0, ESC_PWM_MIN);
    BSP_PWM_SetESC(1, ESC_PWM_MIN);
    HAL_Delay(2000);
    
    // Complete, ESCs should be calibrated now
}
