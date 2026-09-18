#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "rm_aa_control/tracker.hpp"

namespace rm_aa_control {

struct BallisticSolution {
  Eigen::Vector2d angles;  // yaw, pitch [rad]; pitch is positive up
  double flight_time;      // seconds
};

inline BallisticSolution ballistic_pointing_angles(
  const Eigen::Vector3d & target, const Eigen::Vector3d & pitch_to_barrel,
  double bullet_speed, double gravity, int max_iterations = 12,
  double tolerance = 1e-9) {
  
  if (!target.allFinite() || !pitch_to_barrel.allFinite() ||
      !std::isfinite(bullet_speed) || bullet_speed <= 0 ||
      !std::isfinite(gravity) || gravity <= 0 || max_iterations < 1 ||
      max_iterations > 1000 || !std::isfinite(tolerance) || tolerance <= 0) {
    throw std::invalid_argument("Invalid ballistic pointing parameters");
  }

  double yaw = std::atan2(target.y(), target.x());
  double pitch = std::atan2(target.z(), target.head<2>().norm());

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
    const Eigen::Vector3d barrel = rotation * pitch_to_barrel;
    const Eigen::Vector2d horizontal = target.head<2>() - barrel.head<2>();
    const double distance = horizontal.norm();
    const double height = target.z() - barrel.z();
    
    if (distance <= 1e-9) {
      throw std::runtime_error("Target is inside the barrel");
    }

    const double ballistic_coefficient = gravity * distance * distance /
      (2 * bullet_speed * bullet_speed);
    const double discriminant = distance * distance - 4 * ballistic_coefficient *
      (ballistic_coefficient + height);
    
    if (discriminant < 0) {
      throw std::runtime_error("Target is outside the ballistic reach");
    }

    const double root = std::sqrt(discriminant);
    const double candidates[]{
      std::atan((distance + root) / (2 * ballistic_coefficient)),
      std::atan((distance - root) / (2 * ballistic_coefficient))
    };
    
    double next_pitch = std::numeric_limits<double>::quiet_NaN();
    // Start above every finite candidate so the shortest positive flight wins.
    double flight_time = std::numeric_limits<double>::infinity();
    
    for (double candidate : candidates) {
      const double cosine = std::cos(candidate);
      if (cosine <= 1e-12) continue;
      const double candidate_time = distance / (bullet_speed * cosine);
      if (candidate_time > 0 && candidate_time < flight_time) {
        next_pitch = candidate;
        flight_time = candidate_time;
      }
    }
    
    if (!std::isfinite(next_pitch) || !std::isfinite(flight_time)) {
      throw std::runtime_error("No positive-time ballistic solution");
    }

    const double next_yaw = std::atan2(horizontal.y(), horizontal.x());
    const double yaw_change = std::remainder(next_yaw - yaw, 2 * pi);
    
    if (std::abs(yaw_change) < tolerance &&
        std::abs(next_pitch - pitch) < tolerance) {
      return {{next_yaw, next_pitch}, flight_time};
    }
    
    yaw = next_yaw;
    pitch = next_pitch;
  }
  
  throw std::runtime_error("Ballistic pointing iteration did not converge");
}

}  // namespace rm_aa_control
