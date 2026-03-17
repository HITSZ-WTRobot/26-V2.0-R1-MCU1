#include "vision_micro_align.hpp"

#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

VisionMicroAligner::VisionMicroAligner()
    : cfg_{
          .kp_x = 2.0f,
          .kp_y = 2.0f,
          .kp_yaw = 1.5f,
      .ki_x = 0.3f,
      .ki_y = 0.3f,
      .ki_yaw = 0.02f,
      .kd_x = 0.05f,
      .kd_y = 0.05f,
      .kd_yaw = 0.01f,
      .control_dt = 0.01f,
      .integral_limit_x = 0.5f,
      .integral_limit_y = 0.5f,
      .integral_limit_yaw = 30.0f,
          .max_vx = MAX_VEL,
          .max_vy = MAX_VEL,
          .max_wz = MAX_WZ,
          .pos_tolerance = 0.01f,
          .yaw_tolerance_deg = 1.0f,
      } {}

VisionMicroAligner::VisionMicroAligner(const VisionAlignConfig& cfg) : cfg_(cfg) {}

void VisionMicroAligner::setConfig(const VisionAlignConfig& cfg) {
  cfg_ = cfg;
  resetStateInternal();
}

const VisionAlignConfig& VisionMicroAligner::config() const { return cfg_; }

void VisionMicroAligner::reset() { resetStateInternal(); }

Chassis_Velocity_t VisionMicroAligner::computeCommand(const Pose2D& target,
                                                      const Pose2D& self) const {
  if (isAligned(target, self)) {
    resetStateInternal();
    return {.vx = 0.0f, .vy = 0.0f, .wz = 0.0f};
  }

  const float dx_world = target.x - self.x;
  const float dy_world = target.y - self.y;
  const float yaw_err = normalizeDeg(target.yaw_deg - self.yaw_deg);

  const float self_yaw_rad = self.yaw_deg * (kPi / 180.0f);
  const float c = std::cos(self_yaw_rad);
  const float s = std::sin(self_yaw_rad);

  // 世界坐标误差 -> 车体坐标误差
  const float ex_body = c * dx_world + s * dy_world;
  const float ey_body = -s * dx_world + c * dy_world;

  const float dt = (cfg_.control_dt > 1e-4f) ? cfg_.control_dt : 1e-4f;
  if (!state_initialized_) {
    ex_prev_ = ex_body;
    ey_prev_ = ey_body;
    eyaw_prev_ = yaw_err;
    state_initialized_ = true;
  }

  // Position outer-loop PID in body frame.
  ix_ = limitAbs(ix_ + ex_body * dt, cfg_.integral_limit_x);
  iy_ = limitAbs(iy_ + ey_body * dt, cfg_.integral_limit_y);
  iyaw_ = limitAbs(iyaw_ + yaw_err * dt, cfg_.integral_limit_yaw);

  const float dex = (ex_body - ex_prev_) / dt;
  const float dey = (ey_body - ey_prev_) / dt;
  const float deyaw = (yaw_err - eyaw_prev_) / dt;

  Chassis_Velocity_t cmd = {
      .vx = clamp(cfg_.kp_x * ex_body + cfg_.ki_x * ix_ + cfg_.kd_x * dex,
                  -cfg_.max_vx, cfg_.max_vx),
      .vy = clamp(cfg_.kp_y * ey_body + cfg_.ki_y * iy_ + cfg_.kd_y * dey,
                  -cfg_.max_vy, cfg_.max_vy),
      .wz = clamp(cfg_.kp_yaw * yaw_err + cfg_.ki_yaw * iyaw_ +
                      cfg_.kd_yaw * deyaw,
                  -cfg_.max_wz, cfg_.max_wz),
  };

  ex_prev_ = ex_body;
  ey_prev_ = ey_body;
  eyaw_prev_ = yaw_err;

  return cmd;
}

bool VisionMicroAligner::isAligned(const Pose2D& target, const Pose2D& self) const {
  const float dx = target.x - self.x;
  const float dy = target.y - self.y;
  const float pos_err = std::sqrt(dx * dx + dy * dy);
  const float yaw_err = std::fabs(normalizeDeg(target.yaw_deg - self.yaw_deg));

  return (pos_err <= cfg_.pos_tolerance) && (yaw_err <= cfg_.yaw_tolerance_deg);
}

float VisionMicroAligner::clamp(const float value, const float min_value,
                                const float max_value) {
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

float VisionMicroAligner::normalizeDeg(float deg) {
  while (deg > 180.0f)
    deg -= 360.0f;
  while (deg < -180.0f)
    deg += 360.0f;
  return deg;
}

float VisionMicroAligner::limitAbs(const float value, const float abs_limit) {
  const float lim = (abs_limit >= 0.0f) ? abs_limit : -abs_limit;
  return clamp(value, -lim, lim);
}

void VisionMicroAligner::resetStateInternal() const {
  state_initialized_ = false;
  ex_prev_ = 0.0f;
  ey_prev_ = 0.0f;
  eyaw_prev_ = 0.0f;
  ix_ = 0.0f;
  iy_ = 0.0f;
  iyaw_ = 0.0f;
}
