/**
 ******************************************************************************
 * @file    data_hub.c
 * @brief   数据中心 - 实现任务间数据共享 (线程安全)
 ******************************************************************************
 */

#include "data_hub.h"
#include "cmsis_os.h"
#include <string.h>

/* ======================== 私有变量 ======================== */

// 数据存储
static SensorData_t g_SensorData = {0};
static TelemetryData_t g_TelemetryData = {0};
static Command_t g_Command = {0};
static ControlOutput_t g_ControlOutput = {0};

// 互斥锁 (用于数据保护)
static osMutexId_t g_SensorMutex;
static osMutexId_t g_TelemetryMutex;
static osMutexId_t g_CommandMutex;
static osMutexId_t g_OutputMutex;

/* ======================== 公共函数 ======================== */

void DataHub_Init(void)
{
    // 创建互斥锁
    g_SensorMutex = osMutexNew(NULL);
    g_TelemetryMutex = osMutexNew(NULL);
    g_CommandMutex = osMutexNew(NULL);
    g_OutputMutex = osMutexNew(NULL);
    
    // 初始化数据
    memset(&g_SensorData, 0, sizeof(g_SensorData));
    memset(&g_TelemetryData, 0, sizeof(g_TelemetryData));
    memset(&g_Command, 0, sizeof(g_Command));
    memset(&g_ControlOutput, 0, sizeof(g_ControlOutput));
}

/* ========== 传感器数据接口 ========== */

void DataHub_WriteSensor(const SensorData_t *sensor)
{
    if (sensor == NULL) return;
    
    osMutexAcquire(g_SensorMutex, osWaitForever);
    memcpy(&g_SensorData, sensor, sizeof(SensorData_t));
    osMutexRelease(g_SensorMutex);
}

bool DataHub_ReadSensor(SensorData_t *sensor)
{
    bool valid;
    if (sensor == NULL) return false;
    
    osMutexAcquire(g_SensorMutex, osWaitForever);
    memcpy(sensor, &g_SensorData, sizeof(SensorData_t));
    valid = g_SensorData.valid;
    osMutexRelease(g_SensorMutex);
    
    return valid;
}

/* ========== 遥测数据接口 ========== */

void DataHub_WriteTelemetry(const TelemetryData_t *telem)
{
    if (telem == NULL) return;
    
    osMutexAcquire(g_TelemetryMutex, osWaitForever);
    memcpy(&g_TelemetryData, telem, sizeof(TelemetryData_t));
    osMutexRelease(g_TelemetryMutex);
}

bool DataHub_ReadTelemetry(TelemetryData_t *telem)
{
    bool valid;
    if (telem == NULL) return false;
    
    osMutexAcquire(g_TelemetryMutex, osWaitForever);
    memcpy(telem, &g_TelemetryData, sizeof(TelemetryData_t));
    valid = g_TelemetryData.valid;
    osMutexRelease(g_TelemetryMutex);
    
    return valid;
}

/* ========== 命令接口 ========== */

void DataHub_WriteCommand(const Command_t *cmd)
{
    if (cmd == NULL) return;
    
    osMutexAcquire(g_CommandMutex, osWaitForever);
    memcpy(&g_Command, cmd, sizeof(Command_t));
    osMutexRelease(g_CommandMutex);
}

bool DataHub_ReadCommand(Command_t *cmd)
{
    bool valid;
    if (cmd == NULL) return false;
    
    osMutexAcquire(g_CommandMutex, osWaitForever);
    memcpy(cmd, &g_Command, sizeof(Command_t));
    valid = (g_Command.timestamp > 0);
    osMutexRelease(g_CommandMutex);
    
    return valid;
}

/* ========== 控制输出接口 ========== */

void DataHub_WriteOutput(const ControlOutput_t *output)
{
    if (output == NULL) return;
    
    osMutexAcquire(g_OutputMutex, osWaitForever);
    memcpy(&g_ControlOutput, output, sizeof(ControlOutput_t));
    osMutexRelease(g_OutputMutex);
}

bool DataHub_ReadOutput(ControlOutput_t *output)
{
    bool valid;
    if (output == NULL) return false;
    
    osMutexAcquire(g_OutputMutex, osWaitForever);
    memcpy(output, &g_ControlOutput, sizeof(ControlOutput_t));
    valid = (g_ControlOutput.timestamp > 0);
    osMutexRelease(g_OutputMutex);
    
    return valid;
}
