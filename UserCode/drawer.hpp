#pragma once

#include <cstdint>
#include <cmath>

#include "device.hpp"
#include "can.h"
#include "cmsis_os2.h"
#include "eventflags.hpp"

typedef struct
{
    float    drawer_pushout_speed_abs;
    float    drawer_pushout_start_vel_th; // 判定开始运动阈值
    float    drawer_pushout_stall_vel_th;
    float    drawer_pushout_stall_out_th;
    uint32_t drawer_pushout_min_run_ms;       // 开始运行后什么时候开始堵转检测
    uint32_t drawer_pushout_stall_confirm_ms; // 堵转确认时间
} DRAWER_PUSHOUT_PARAMS_T;

typedef enum
{
    MODE_UP = 0, // 只要不在抽屉向下复位状态就在MODE_UP状态
    MODE_RESET_DOWN
} RUN_MODE_E;

typedef enum
{
    DRAWER_IN = 0,  // 抽屉收起了
    DRAWER_OUT,     // 抽屉推出了
    DRAWER_UP,      // 抽屉在顶端
    DRAWER_RUNNING, // 抽屉在运动中
    DRAWER_UNKNOWN  // 抽屉状态未知（常常在开始时）
} DRAWER_STATUS_E;  // 描述抽屉的状态

void Drawer_TIM_Callback(void);
void DrawerUP_Control(void* argument);
void DrawerOUT_Control(void* argument);
void Drawer_Init(void);
void Drawer_softTIM(void* arguement);