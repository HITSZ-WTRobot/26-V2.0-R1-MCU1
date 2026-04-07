#include "chassis.hpp"

#include "JustEncoder.hpp"
#include "device.hpp"
#include "main.h"

#include <cstddef>

namespace {

constexpr float kControlDt = 0.001f;

// 轮子的pid参数
constexpr PIDMotor::Config motor_wheel_vel_pid = {
    .Kp = 100.0f, .Ki = 0.80f, .Kd = 1.0f, .abs_output_max = 8000.0f};

constexpr velocity_profile::SCurveProfile::Config kTranslationLimit = {
    .max_spd = 1.0f,
    .max_acc = 0.5f,
    .max_jerk = 1.0f,
};

constexpr velocity_profile::SCurveProfile::Config kRotationLimit = {
  .max_spd = 90.0f,
  .max_acc = 120.0f,
  .max_jerk = 720.0f,
};
chassis::motion::Omni4* omni4;
controllers::MotorVelController *motor_vel_ctrl[4]{};

float target_x_last = 0.0f;
float target_y_last = 0.0f;
float target_yaw_last = 0.0f;

} // namespace

ChassisLoc*        loc_encoder;
ChassisController* chassis_;

float target_x = 0.0f;
float target_y = 0.0f;
float target_yaw = 0.0f;
Control_Mode chassis_control_mode = VEL_Control;
Chassis_Velocity_t chassis_v = {0.0f, 0.0f, 0.0f};

void APP_Chassis_BeforeUpdate() {
  using chassis::motion::Omni4;
  using controllers::MotorVelController;

  for (size_t i = 0; i < 4; ++i)
    motor_vel_ctrl[i] =
        new MotorVelController(motor_wheel[i], {.pid = motor_wheel_vel_pid});

  omni4 = new Omni4(
            {
                .wheel_radius = 63.5f,
                .wheel_distance_x = 820.23f,
                .wheel_distance_y = 840.42f,
                .wheel_front_right = motor_vel_ctrl[0],
                .wheel_front_left = motor_vel_ctrl[1],
                .wheel_rear_left = motor_vel_ctrl[2],
                .wheel_rear_right = motor_vel_ctrl[3],
            });
    
  loc_encoder = new ChassisLoc(*omni4);

  chassis_ = new ChassisController(*omni4, *loc_encoder,
      {
          .posture_error_pd_cfg =
              {
                  .vx = {.Kp = 2.0f, .Kd = 1.0f, .abs_output_max = 0.1f},
                  .vy = {.Kp = 2.0f, .Kd = 1.0f, .abs_output_max = 0.1f},
                    .wz = {.Kp = 4.0f, .Kd = 1.5f, .abs_output_max = 60.0f},
              },
          .limit =
              {
                  .x = kTranslationLimit,
                  .y = kTranslationLimit,
                  .yaw = kRotationLimit,
              },
      });
}

void APP_Chassis_InitBeforeUpdate() { APP_Chassis_BeforeUpdate(); }

void APP_Chassis_Init() {
  if (!chassis_ || !loc_encoder)
    Error_Handler();

  if (!chassis_->enable())
    Error_Handler();

  target_x = 0.0f;
  target_y = 0.0f;
  target_yaw = 0.0f;
  target_x_last = 0.0f;
  target_y_last = 0.0f;
  target_yaw_last = 0.0f;
  chassis_v = {0.0f, 0.0f, 0.0f};
  chassis_control_mode = VEL_Control;
}

void Chassis_TIM_Callback() {
  if (!chassis_ || !loc_encoder || !chassis_->enabled())
    return;

  loc_encoder->update(kControlDt);

  switch (chassis_control_mode) {
  case POS_Control:
    if (target_x != target_x_last || target_y != target_y_last ||
        target_yaw != target_yaw_last)
      APP_Chassis_Move(target_x, target_y, target_yaw);

    target_x_last = target_x;
    target_y_last = target_y;
    target_yaw_last = target_yaw;
    break;

  case VEL_Control:
  default:
    chassis_->setVelocityInBody(
        {
            .vx = chassis_v.vx,
            .vy = chassis_v.vy,
            .wz = chassis_v.wz,
        },
        false);
    break;
  }

  chassis_->profileUpdate(kControlDt);
  chassis_->errorUpdate();
  chassis_->controllerUpdate();
  omni4->update();
}

void APP_Chassis_Update_1kHz() { Chassis_TIM_Callback(); }

void APP_Chassis_Update_100Hz() { Chassis_TIM_Callback(); }
