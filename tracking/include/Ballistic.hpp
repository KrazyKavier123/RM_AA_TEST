#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "Tracker.hpp"

namespace rmv {

// 弹道指向解。移植自 src/rm_aa_control/include/rm_aa_control/ballistic.hpp。
struct BallisticSolution {
    Eigen::Vector2d angles;  // [yaw, pitch]，单位 rad，pitch 向上为正
    double flight_time{0};   // 飞行时间 (s)
};

// target/muzzle 都在云台系（x 前、y 左、z 上，单位 m）。
inline BallisticSolution ballistic_angles(const Eigen::Vector3d& target,
                                          const Eigen::Vector3d& muzzle,
                                          double bullet_speed, double gravity,
                                          int max_iter = 12, double tol = 1e-9) {
    double yaw = std::atan2(target.y(), target.x());
    double pitch = std::atan2(target.z(), target.head<2>().norm());

    for (int iter = 0; iter < max_iter; ++iter) {
        const Eigen::Matrix3d rot =
            (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()))
                .toRotationMatrix();
        const Eigen::Vector3d barrel = rot * muzzle;  // 枪口随 yaw/pitch 转动
        const Eigen::Vector2d horizontal = target.head<2>() - barrel.head<2>();
        const double dist = horizontal.norm();
        const double height = target.z() - barrel.z();
        if (dist <= 1e-9) throw std::runtime_error("目标位于枪口处");

        // 解 d*tan^2 - ... 的一元二次：取正根中最短的飞行时间。
        const double k = gravity * dist * dist / (2 * bullet_speed * bullet_speed);
        const double disc = dist * dist - 4 * k * (k + height);
        if (disc < 0) throw std::runtime_error("目标超出弹道射程");
        const double root = std::sqrt(disc);
        const double cand[2] = {std::atan((dist + root) / (2 * k)),
                                std::atan((dist - root) / (2 * k))};

        double next_pitch = std::numeric_limits<double>::quiet_NaN();
        double flight = std::numeric_limits<double>::infinity();
        for (double c : cand) {
            const double cosine = std::cos(c);
            if (cosine <= 1e-12) continue;
            const double time = dist / (bullet_speed * cosine);
            if (time > 0 && time < flight) {
                next_pitch = c;
                flight = time;
            }
        }
        if (!std::isfinite(next_pitch)) throw std::runtime_error("无有效弹道解");

        const double next_yaw = std::atan2(horizontal.y(), horizontal.x());
        if (std::abs(wrap(next_yaw - yaw)) < tol && std::abs(next_pitch - pitch) < tol)
            return {{next_yaw, next_pitch}, flight};
        yaw = next_yaw;
        pitch = next_pitch;
    }
    throw std::runtime_error("弹道迭代未收敛");
}

}  // namespace rmv
