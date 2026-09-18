#pragma once

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace rm_aa_control {
// Intersecting yaw/pitch axes at the base origin; pitch is positive up.
// offset is P-axis -> optical centre in the rotating pitch frame (forward/left/up).
// The optical axis is parallel to pitch-frame +X. All lengths are in metres.
inline Eigen::Vector2d camera_pointing_angles(
    const Eigen::Vector3d & target, const Eigen::Vector3d & offset) {
  if (!target.allFinite() || !offset.allFinite())
    throw std::invalid_argument("Nonfinite camera pointing geometry");
  const double horizontal_squared = target.head<2>().squaredNorm() - offset.y() * offset.y();
  if (horizontal_squared <= 1e-12)
    throw std::runtime_error("Undefined or unreachable camera yaw");
  const double horizontal = std::sqrt(horizontal_squared);
  const double forward_squared = horizontal_squared + target.z() * target.z() -
    offset.z() * offset.z();
  if (forward_squared <= 1e-12 || std::sqrt(forward_squared) - offset.x() <= 1e-6)
    throw std::runtime_error("Target outside forward camera pointing branch");
  const double yaw = std::atan2(target.y(), target.x()) -
    std::atan2(offset.y(), horizontal);
  const double pitch = std::atan2(target.z(), horizontal) -
    std::atan2(offset.z(), std::sqrt(forward_squared));
  return {yaw, pitch};
}
}  // namespace rm_aa_control
