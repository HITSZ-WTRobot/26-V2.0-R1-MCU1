#pragma once

#include "chassis.hpp"

struct Pose2D {
  float x;
  float y;
  float yaw_deg;
};

struct VisionAlignConfig {
  float kp_x;
  float kp_y;
  float kp_yaw;

  float max_vx;
  float max_vy;
  float max_wz;

  float pos_tolerance;
  float yaw_tolerance_deg;
};

class VisionMicroAligner {
public:
  VisionMicroAligner();
  explicit VisionMicroAligner(const VisionAlignConfig& cfg);

  void setConfig(const VisionAlignConfig& cfg);
  const VisionAlignConfig& config() const;

  // 输入：目标位姿 + 自身位姿（同一世界坐标系，角度单位为度）
  // 输出：底盘体坐标速度指令 vx/vy/wz
  Chassis_Velocity_t computeCommand(const Pose2D& target, const Pose2D& self) const;

  // 判断是否已经完成对齐
  bool isAligned(const Pose2D& target, const Pose2D& self) const;

private:
  static float clamp(float value, float min_value, float max_value);
  static float normalizeDeg(float deg);

  VisionAlignConfig cfg_;
};
