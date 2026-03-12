#pragma once

#include "dji.hpp"

extern motors::DJIMotor* motor_drawer_1;
extern motors::DJIMotor* motor_drawer_2;
extern motors::DJIMotor* motor_drawer_3;
extern motors::DJIMotor* motor_drawer_4;
extern motors::DJIMotor* motor_wheel[4];
extern motors::DJIMotor* motor_drawer_out;

void APP_Device_Init(void);
void APP_Device_Update(void);
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();