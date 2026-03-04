#pragma once

#include "dji.hpp"

extern motors::DJIMotor* drawer_1;
extern motors::DJIMotor* drawer_2;
extern motors::DJIMotor* drawer_3;
extern motors::DJIMotor* drawer_4;
extern motors::DJIMotor* motor_wheel[4];
extern motors::DJIMotor* drawer_out;

void APP_Device_Init(void);
void APP_Device_Update(void);
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();