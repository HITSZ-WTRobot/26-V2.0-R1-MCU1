#pragma once

#include "chassis.hpp"

struct Pose2D {
  float x;
  float y;
  float yaw_deg;
};

struct VisionAlignConfig {
  float kp_x = 0.0f;
  float kp_y = 0.0f;
  float kp_yaw = 0.0f;

  float ki_x = 0.0f;
  float ki_y = 0.0f;
  float ki_yaw = 0.0f;

  float kd_x = 0.0f;
  float kd_y = 0.0f;
  float kd_yaw = 0.0f;

  float control_dt = 0.01f;
  float integral_limit_x = 0.5f;
  float integral_limit_y = 0.5f;
  float integral_limit_yaw = 30.0f;

  float max_vx = 0.0f;
  float max_vy = 0.0f;
  float max_wz = 0.0f;

  float pos_tolerance = 0.0f;
  float yaw_tolerance_deg = 0.0f;
};

class VisionMicroAligner {
public:
  VisionMicroAligner();
  explicit VisionMicroAligner(const VisionAlignConfig& cfg);

  void setConfig(const VisionAlignConfig& cfg);
  const VisionAlignConfig& config() const;
  void reset();

  // 输入：目标位姿 + 自身位姿（同一世界坐标系，角度单位为度）
  // 输出：底盘体坐标速度指令 vx/vy/wz
  Chassis_Velocity_t computeCommand(const Pose2D& target, const Pose2D& self) const;

  // 判断是否已经完成对齐
  bool isAligned(const Pose2D& target, const Pose2D& self) const;

private:
  static float clamp(float value, float min_value, float max_value);
  static float normalizeDeg(float deg);
  static float limitAbs(float value, float abs_limit);

  void resetStateInternal() const;

  VisionAlignConfig cfg_;

  // Mutable state enables periodic closed-loop update while keeping API unchanged.
  mutable bool state_initialized_ = false;
  mutable float ex_prev_ = 0.0f;
  mutable float ey_prev_ = 0.0f;
  mutable float eyaw_prev_ = 0.0f;
  mutable float ix_ = 0.0f;
  mutable float iy_ = 0.0f;
  mutable float iyaw_ = 0.0f;
};
