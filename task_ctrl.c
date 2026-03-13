/**
 ******************************************************************************
 * @file    task_ctrl.c
 * @brief   核心控制任务 - 100Hz 周期循环
 * @note    最高优先级任务,负责闭环控制逻辑
 ******************************************************************************
 */

#include "task_ctrl.h"
#include "data_hub.h"
#include "bsp_pwm.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/* ======================== 配置参数 ======================== */
#define CTRL_TASK_FREQ_HZ       100     // 控制频率 100Hz
#define CTRL_TASK_PERIOD_MS     (1000 / CTRL_TASK_FREQ_HZ)

// PID 参数 (示例,需根据实际调试)
#define PID_DEPTH_KP    50.0f
#define PID_DEPTH_KI    0.5f
#define PID_DEPTH_KD    10.0f

#define PID_PITCH_KP    2.0f
#define PID_PITCH_KI    0.1f
#define PID_PITCH_KD    0.5f
#define VALVE_CTRL_GPIO_Port GPIOE
/* ======================== 私有类型 ======================== */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_limit;
} PID_t;

/* ======================== 私有变量 ======================== */
static PID_t g_PID_Depth;
static PID_t g_PID_Pitch;
static PID_t g_PID_Yaw;

/* ======================== 私有函数 ======================== */

/**
 * @brief PID 初始化
 */
static void PID_Init(PID_t *pid, float kp, float ki, float kd, float limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = limit;
}

/**
 * @brief PID 计算
 * @param pid PID 结构体指针
 * @param setpoint 目标值
 * @param feedback 反馈值
 * @param dt 时间间隔 (秒)
 * @return 控制输出
 */
static float PID_Compute(PID_t *pid, float setpoint, float feedback, float dt)
{
    float error = setpoint - feedback;
    float derivative, output;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅 (防止积分饱和)
    
    if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
    if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;
    
    // 微分项
    derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // PID 输出
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}

/**
 * @brief 控制量映射到 PWM 值
 * @param ctrl_value 控制量 (-1.0 ~ 1.0)
 * @return PWM 占空比 (500 ~ 2500 对应 1-2ms 脉宽)
 */
static uint16_t MapToPWM(float ctrl_value)
{
    uint16_t pwm;
    
    // 限幅
    if (ctrl_value > 1.0f) ctrl_value = 1.0f;
    if (ctrl_value < -1.0f) ctrl_value = -1.0f;
    
    // 映射: -1.0 -> 500, 0.0 -> 1500, 1.0 -> 2500
    pwm = (uint16_t)(1500 + ctrl_value * 1000);
    return pwm;
}

/**
 * @brief 自动控制模式 - 深度/姿态控制
 */
static void AutoControl(const SensorData_t *sensor, const Command_t *cmd, ControlOutput_t *output)
{
    const float dt = 0.01f;  // 100Hz -> 10ms
    
    // 深度控制 (输出俯仰角指令)
    float pitch_cmd = PID_Compute(&g_PID_Depth, cmd->target_depth, sensor->depth_m, dt);
    
    // 俯仰角控制 (输出升降舵角度)
    float elevator = PID_Compute(&g_PID_Pitch, pitch_cmd, sensor->pitch, dt);
    
    // 航向控制 (输出方向舵角度)
    float rudder = PID_Compute(&g_PID_Yaw, cmd->target_yaw, sensor->yaw, dt);
    
    // 映射到 PWM
    output->servo_pwm[0] = MapToPWM(elevator);   // 升降舵
    output->servo_pwm[1] = MapToPWM(rudder);     // 方向舵
    output->servo_pwm[2] = 1500;                 // 横滚舵中立
    
    // 推进器保持恒定转速 (可根据需要调整)
    output->esc_pwm[0] = 1600;  // 左推进器
    output->esc_pwm[1] = 1600;  // 右推进器
}

/**
 * @brief 手动控制模式
 */
static void ManualControl(const Command_t *cmd, ControlOutput_t *output)
{
    // 直接映射指令到 PWM
    output->servo_pwm[0] = MapToPWM(cmd->pitch_cmd);
    output->servo_pwm[1] = MapToPWM(cmd->yaw_cmd);
    output->servo_pwm[2] = MapToPWM(cmd->roll_cmd);
    
    output->esc_pwm[0] = MapToPWM(cmd->thrust_cmd[0]);
    output->esc_pwm[1] = MapToPWM(cmd->thrust_cmd[1]);
}

/* ======================== 任务入口 ======================== */

void StartCtrlTask(void *argument)
{
    SensorData_t sensor;
    Command_t cmd;
    ControlOutput_t output;
    int i;
			
//	printf("StartCtrlTask\n");
    output.ctrl_mode = 0;
    
    // 初始化 PID 控制器
    PID_Init(&g_PID_Depth, PID_DEPTH_KP, PID_DEPTH_KI, PID_DEPTH_KD, 30.0f);
    PID_Init(&g_PID_Pitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 1.0f);
    PID_Init(&g_PID_Yaw, 2.0f, 0.05f, 0.3f, 1.0f);
    
    for(;;)
    {
				#ifdef SIMULATOR_MODE
					printf("%s\n", __FUNCTION__);
				#endif
        // ========== 1. 读取数据 ==========
        DataHub_ReadSensor(&sensor);
        DataHub_ReadCommand(&cmd);
        
        // ========== 2. 控制逻辑 ==========
        output.ctrl_mode = cmd.mode;
        output.timestamp = HAL_GetTick();
        
        switch (cmd.mode) {
            case 0:  // 待机模式
                // 舵机中立位, 推进器停止
                for (i = 0; i < MAX_SERVOS; i++) {
                    output.servo_pwm[i] = 1500;
                }
                for (i = 0; i < MAX_ESCS; i++) {
                    output.esc_pwm[i] = 1000;  // ESC 最小油门
                }
                break;
                
            case 1:  // 手动模式
                ManualControl(&cmd, &output);
                break;
                
            case 2:  // 自动模式
                AutoControl(&sensor, &cmd, &output);
                break;
                
            default:
                break;
        }
        
        // 电磁阀控制
        output.valve_state.val = cmd.valve_open;
        
        // ========== 3. 输出执行 ==========
        // 更新 PWM 输出
        BSP_PWM_SetServo(0, output.servo_pwm[0]);
        BSP_PWM_SetServo(1, output.servo_pwm[1]);
        BSP_PWM_SetServo(2, output.servo_pwm[2]);
        BSP_PWM_SetESC(0, output.esc_pwm[0]);
        BSP_PWM_SetESC(1, output.esc_pwm[1]);

        // 控制电磁阀 GPIO
        HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve1_Pin, output.valve_state.bstate.valve_state1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve2_Pin, output.valve_state.bstate.valve_state2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve3_Pin, output.valve_state.bstate.valve_state3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve4_Pin, output.valve_state.bstate.valve_state4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
				
				HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve5_Pin, output.valve_state.bstate.valve_state5 ? GPIO_PIN_SET : GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VALVE_CTRL_GPIO_Port, ballValve6_Pin, output.valve_state.bstate.valve_state6 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        // 写入 DataHub (供上位机读取)
        DataHub_WriteOutput(&output);
        
        // ========== 4. 任务同步 ==========
        // 严格 100Hz 周期执行 (使用相对延时)
        osDelay(CTRL_TASK_PERIOD_MS);
    }
}

/* ======================== 调试接口 ======================== */

/**
 * @brief 获取 PID 参数 (用于在线调参)
 */
void CtrlTask_GetPIDParams(uint8_t pid_id, float *kp, float *ki, float *kd)
{
    PID_t *pid = NULL;
    switch (pid_id) {
        case 0: pid = &g_PID_Depth; break;
        case 1: pid = &g_PID_Pitch; break;
        case 2: pid = &g_PID_Yaw; break;
        default: return;
    }
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
}

/**
 * @brief 设置 PID 参数
 */
void CtrlTask_SetPIDParams(uint8_t pid_id, float kp, float ki, float kd)
{
    PID_t *pid = NULL;
    switch (pid_id) {
        case 0: pid = &g_PID_Depth; break;
        case 1: pid = &g_PID_Pitch; break;
        case 2: pid = &g_PID_Yaw; break;
        default: return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
