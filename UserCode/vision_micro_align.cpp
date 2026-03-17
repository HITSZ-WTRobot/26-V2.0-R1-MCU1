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
          .max_vx = MAX_VEL,
          .max_vy = MAX_VEL,
          .max_wz = MAX_WZ,
          .pos_tolerance = 0.01f,
          .yaw_tolerance_deg = 1.0f,
      } {}

VisionMicroAligner::VisionMicroAligner(const VisionAlignConfig& cfg) : cfg_(cfg) {}

void VisionMicroAligner::setConfig(const VisionAlignConfig& cfg) { cfg_ = cfg; }

const VisionAlignConfig& VisionMicroAligner::config() const { return cfg_; }

Chassis_Velocity_t VisionMicroAligner::computeCommand(const Pose2D& target,
                                                      const Pose2D& self) const {
  if (isAligned(target, self)) {
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

  Chassis_Velocity_t cmd = {
      .vx = clamp(cfg_.kp_x * ex_body, -cfg_.max_vx, cfg_.max_vx),
      .vy = clamp(cfg_.kp_y * ey_body, -cfg_.max_vy, cfg_.max_vy),
      .wz = clamp(cfg_.kp_yaw * yaw_err, -cfg_.max_wz, cfg_.max_wz),
  };
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
