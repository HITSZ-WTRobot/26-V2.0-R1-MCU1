#pragma once

#include "../Modules/ChassisController/Controller/Master/Master.hpp"
#include "Omni4.hpp"
#include "cmsis_os2.h"

#include <cmath>

#define MAX_VEL 1.0f
#define MAX_WZ 90.0f

using Chassis = chassis::controller::Master<chassis::Omni4>;

enum Control_Mode {
  POS_Control = 0,
  VEL_Control = 1,
};

struct Chassis_Velocity_t {
  float vx;
  float vy;
  float wz;
};

extern Chassis *chassis_;
extern Chassis_Velocity_t chassis_v;

extern float target_x;
extern float target_y;
extern float target_yaw;
extern Control_Mode chassis_control_mode;

void APP_Chassis_BeforeUpdate();
void APP_Chassis_InitBeforeUpdate();
void APP_Chassis_Init();
void APP_Chassis_Update_100Hz();
void APP_Chassis_Update_1kHz();
void Chassis_TIM_Callback();

inline void APP_Chassis_Update() { APP_Chassis_Update_1kHz(); }

inline void APP_Chassis_MoveTo(const float x, const float y) {
  if (!chassis_)
    return;

  (void)chassis_->setTargetPostureInWorld({
      .x = x,
      .y = y,
      .yaw = chassis_->posture().in_world.yaw,
  });
}

inline void APP_Chassis_MoveToSync(const float x, const float y) {
  if (!chassis_)
    return;

  APP_Chassis_MoveTo(x, y);
  while (!chassis_->isTrajectoryFinished())
    osDelay(1);

  chassis_->setVelocityInBody({.vx = 0.0f, .vy = 0.0f, .wz = 0.0f}, false);
}

inline void APP_Chassis_Move(const float x, const float y, const float yaw) {
  if (!chassis_)
    return;

  (void)chassis_->setTargetPostureInBody({
      .x = x,
      .y = y,
      .yaw = yaw,
  });
}

inline void APP_Chassis_Rotate(const float yaw) {
  APP_Chassis_Move(0.0f, 0.0f, yaw);
}

inline void APP_Chassis_RotateSync(const float yaw) {
  if (!chassis_)
    return;

  APP_Chassis_Rotate(yaw);
  while (!chassis_->isTrajectoryFinished())
    osDelay(1);

  chassis_->setVelocityInBody({.vx = 0.0f, .vy = 0.0f, .wz = 0.0f}, false);
}
