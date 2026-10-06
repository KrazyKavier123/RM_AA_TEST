#include "Tracker.hpp"

#include <algorithm>
#include <limits>

namespace rmv {

Tracker::Tracker(TrackerConfig config) : cfg_(config) {}

void Tracker::reset() {
    init_ = false;
    updates_ = 0;
    sel_ = -1;
}

bool Tracker::valid(double stamp) const {
    return init_ && stamp >= t_ && stamp - seen_ <= cfg_.timeout;
}

Tracker::State Tracker::state_at(double stamp) const {
    State result = x_;
    const double dt = stamp - t_;
    for (int i : {0, 2, 4}) result[i] += dt * result[i + 1];
    result[6] = wrap(x_[6] + dt * x_[7] + 0.5 * dt * dt * x_[8]);
    result[7] = x_[7] + dt * x_[8];
    return result;
}

TrackObs Tracker::measure(const State& x, int index) const {
    const double a = wrap(x[6] + 2.0 * kPi * index / cfg_.plates);
    const bool odd = cfg_.alt_geometry && index % 2 != 0;
    const double radius = x[odd ? 10 : 9];
    return {{x[0] - radius * std::cos(a), x[2] - radius * std::sin(a),
             x[4] + (odd ? x[11] : 0.0)},
            a};
}

TrackObs Tracker::plate_at(int index, double stamp) const {
    return measure(state_at(stamp), index);
}

Eigen::Matrix<double, 4, 12> Tracker::jacobian(int index) const {
    Eigen::Matrix<double, 4, 12> h = Eigen::Matrix<double, 4, 12>::Zero();
    const bool odd = cfg_.alt_geometry && index % 2 != 0;
    const int r = odd ? 10 : 9;
    const double a = x_[6] + 2.0 * kPi * index / cfg_.plates;
    h(0, 0) = h(1, 2) = h(2, 4) = h(3, 6) = 1;
    h(0, 6) = x_[r] * std::sin(a);
    h(1, 6) = -x_[r] * std::cos(a);
    h(0, r) = -std::cos(a);
    h(1, r) = -std::sin(a);
    if (odd) h(2, 11) = 1;
    return h;
}

Tracker::Cov Tracker::cov_at(double stamp) const {
    const double dt = stamp - t_;
    Cov f = Cov::Identity(), q = Cov::Zero();
    for (int i : {0, 2, 4}) {
        f(i, i + 1) = dt;
        const double var = cfg_.acc_var;
        q(i, i) = var * std::pow(dt, 4) / 4;
        q(i, i + 1) = q(i + 1, i) = var * std::pow(dt, 3) / 2;
        q(i + 1, i + 1) = var * dt * dt;
    }
    f(6, 7) = dt;
    f(6, 8) = 0.5 * dt * dt;
    f(7, 8) = dt;
    // 匀角加速度模型：每步内恒定的随机角加加速度驱动 yaw/rate/acc。
    const Eigen::Vector3d g(dt * dt * dt / 6.0, dt * dt / 2.0, dt);
    q.block<3, 3>(6, 6) = cfg_.yaw_jerk_var * g * g.transpose();
    q(9, 9) = 1e-5 * dt;
    if (cfg_.alt_geometry) q(10, 10) = q(11, 11) = 1e-5 * dt;
    return f * P_ * f.transpose() + q;
}

bool Tracker::correct(const TrackObs& obs, int index) {
    const TrackObs expected = measure(x_, index);
    Eigen::Vector4d residual;
    residual.head<3>() = obs.pos - expected.pos;
    residual[3] = wrap(obs.yaw - expected.yaw);

    const Eigen::Matrix<double, 4, 12> h = jacobian(index);
    Eigen::Matrix4d r = Eigen::Matrix4d::Identity() * cfg_.pos_var;
    r(3, 3) = cfg_.yaw_var;
    const Eigen::Matrix4d s = h * P_ * h.transpose() + r;
    const Eigen::LDLT<Eigen::Matrix4d> factor(s);
    if (factor.info() != Eigen::Success || !factor.isPositive()) return false;

    const double nis = residual.dot(factor.solve(residual));
    if (!std::isfinite(nis) || nis > cfg_.nis_gate) return false;  // 离群观测

    const Eigen::Matrix<double, 12, 4> k = factor.solve(h * P_).transpose();
    State next = x_ + k * residual;
    if (!next.allFinite() || next[9] < cfg_.min_radius || next[9] > cfg_.max_radius ||
        next[10] < cfg_.min_radius || next[10] > cfg_.max_radius ||
        std::abs(next[11]) > cfg_.max_dz)
        return false;
    next[6] = wrap(next[6]);

    // Joseph 形式保持协方差对称正定。
    const Cov a = Cov::Identity() - k * h;
    P_ = a * P_ * a.transpose() + k * r * k.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    x_ = next;
    return true;
}

bool Tracker::update(const std::vector<TrackObs>& obs, double stamp,
                     const Eigen::Vector3d& camera) {
    if (!std::isfinite(stamp) || !camera.allFinite()) return false;
    if (init_ && stamp <= t_) return false;  // 时间倒流：拒绝
    if (init_ && stamp - seen_ > cfg_.timeout) reset();

    std::vector<TrackObs> good;
    for (const TrackObs& o : obs)
        if (o.pos.allFinite() && std::isfinite(o.yaw)) good.push_back(o);

    if (!init_) {
        if (good.empty()) return false;
        // 用最近的一帧观测初始化：车体中心 = 装甲板位置 + 半径 * 内法线。
        const auto nearest = std::min_element(good.begin(), good.end(), [&](const auto& a, const auto& b) {
            return (a.pos - camera).squaredNorm() < (b.pos - camera).squaredNorm();
        });
        x_ << nearest->pos.x() + cfg_.radius * std::cos(nearest->yaw), 0,
            nearest->pos.y() + cfg_.radius * std::sin(nearest->yaw), 0, nearest->pos.z(), 0,
            wrap(nearest->yaw), 0, 0, cfg_.radius, cfg_.radius, 0;
        P_.setIdentity();
        P_.diagonal() << 0.1, 4, 0.1, 4, 0.1, 4, 0.2, 4, cfg_.yaw_acc_var, 0.02, 0.02, 0.02;
        t_ = seen_ = stamp;
        init_ = true;
        sel_ = 0;
        ids_.fill(0);
        if (nearest->id > 0) ids_[0] = nearest->id;
        return true;
    }

    P_ = cov_at(stamp);  // 预测
    x_ = state_at(stamp);
    t_ = stamp;

    struct Pair {
        double cost;
        std::size_t obs;
        int plate;
    };
    std::vector<Pair> pairs;
    // 将 EKF 预测到当前帧的四个装甲板位置和朝向，逐一与 本帧检测后经 PnP 解算的装甲板位置和朝向比较。
    /*
    good[j]：第 j 个实际观测，包含解算后的位置 pos 和朝向角 yaw。
    measure(x_, i)：根据 EKF 当前预测的车体状态，计算第 i 个板位的位置和朝向。
    d：两者的三维位置距离。
    a：两者的朝向角差，wrap() 处理跨越 ±π 的情况。
    两个误差都满足门限，就把这组对应关系加入候选 pairs。
    */
    for (std::size_t j = 0; j < good.size(); ++j)
        for (int i = 0; i < cfg_.plates; ++i) {
            const TrackObs p = measure(x_, i);
            const double d = (good[j].pos - p.pos).norm();
            const double a = std::abs(wrap(good[j].yaw - p.yaw));
            if (d <= cfg_.assoc_dist && a <= cfg_.assoc_yaw)
                pairs.push_back({d / cfg_.assoc_dist + a / cfg_.assoc_yaw, j, i});
        }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.cost < b.cost; });

    std::vector<bool> used_obs(good.size(), false), used_plate(cfg_.plates, false);
    bool accepted = false;
    for (const Pair& p : pairs) {
        if (used_obs[p.obs] || used_plate[p.plate]) continue;
        if (correct(good[p.obs], p.plate)) {
            used_obs[p.obs] = used_plate[p.plate] = true;
            if (good[p.obs].id > 0) ids_[p.plate] = good[p.obs].id;
            accepted = true;
        }
    }
    if (accepted) {
        seen_ = stamp;
        ++updates_;
    }
    return accepted;
}

int Tracker::select_nearest(double stamp, const Eigen::Vector3d& camera) {
    if (!valid(stamp) || !camera.allFinite()) return -1;
    int best = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < cfg_.plates; ++i) {
        const double d = (plate_at(i, stamp).pos - camera).norm();
        if (d < best_distance) {
            best = i;
            best_distance = d;
        }
    }
    // 只有新板明显更近时才换板。
    if (sel_ < 0 || best_distance + cfg_.switch_margin <
                        (plate_at(sel_, stamp).pos - camera).norm()) // 当前选中板到相机的距离
        sel_ = best;
    return sel_;
}

}  // namespace rmv
