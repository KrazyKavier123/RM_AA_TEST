#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <vector>

namespace rmv {

constexpr double kPi = 3.14159265358979323846;
inline double wrap(double angle) { return std::remainder(angle, 2.0 * kPi); }

// 一帧装甲板观测，坐标已变换到云台系（x 前、y 左、z 上，单位 m）。
struct TrackObs {
    Eigen::Vector3d pos;
    double yaw{0};  // 装甲板内法线朝向，指向车体中心
    int id{0};      // 装甲板编号，0 表示未知
};

// 旋转车体 EKF 参数。移植自 src/rm_aa_control/src/tracker.cpp。
struct TrackerConfig {
    int plates{4};                 // 装甲板数量
    double radius{0.25};           // 初始旋转半径 (m)
    double min_radius{0.05};
    double max_radius{0.8};
    double acc_var{4.0};           // 平移加速度方差
    double yaw_acc_var{16.0};      // 角加速度方差
    double pos_var{0.0025};        // 位置观测方差
    double yaw_var{0.04};          // 角度观测方差
    double assoc_dist{0.5};        // 位置关联门限 (m)
    double assoc_yaw{1.0};         // 角度关联门限 (rad)
    double nis_gate{18.47};        // 新息卡方门限
    double timeout{0.3};           // 超时重置 (s)
    double switch_margin{0.05};    // 换板滞回 (m)
    bool alt_geometry{false};      // 奇偶板半径/高度交替（如前哨站）
    double max_dz{0.3};            // 奇偶板高度差上限 (m)
};

class Tracker {
public:
    // 状态向量：[cx, vx, cy, vy, z, vz, yaw, yaw_rate, r_even, r_odd, dz_odd]
    using State = Eigen::Matrix<double, 11, 1>;
    using Cov = Eigen::Matrix<double, 11, 11>;

    explicit Tracker(TrackerConfig config = {});

    void reset();
    // 用一帧观测更新滤波器。stamp 单位 s；camera 为云台系中的相机原点。
    bool update(const std::vector<TrackObs>& obs, double stamp,
                const Eigen::Vector3d& camera);
    bool valid(double stamp) const;                    // 是否有可用目标
    State state_at(double stamp) const;                // 预测到 stamp 的状态
    TrackObs plate_at(int index, double stamp) const;  // 预测某块装甲板
    // 选取离相机最近的装甲板，带换板滞回；返回 -1 表示没有可用目标。
    int select_nearest(double stamp, const Eigen::Vector3d& camera);

    int updates() const { return updates_; }
    int plate_id(int index) const { return ids_.at(index); }

private:
    TrackObs measure(const State& x, int index) const;
    Eigen::Matrix<double, 4, 11> jacobian(int index) const;
    Cov cov_at(double stamp) const;
    bool correct(const TrackObs& obs, int index);

    TrackerConfig cfg_;
    State x_{State::Zero()};
    Cov P_{Cov::Identity()};
    std::array<int, 16> ids_{};  // 每块装甲板最近一次的分类编号
    bool init_{false};
    int updates_{0};
    double t_{0}, seen_{0};
    int sel_{-1};
};

}  // namespace rmv
