#include "rm_aa_control/tracker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rm_aa_control {
double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }
Tracker::Tracker(TrackerConfig config) : config_(config) {
  for (double v : {config.radius, config.min_radius, config.max_radius,
       config.acceleration_variance, config.angular_acceleration_variance,
       config.position_variance, config.yaw_variance, config.association_distance,
       config.association_yaw, config.nis_gate, config.timeout, config.max_height_difference})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid tracker parameter");
  if (config.plate_count < 1 || config.plate_count > 16 ||
      config.radius < config.min_radius || config.radius > config.max_radius ||
      !std::isfinite(config.switch_margin) || config.switch_margin < 0 ||
      (config.alternating_geometry && config.plate_count % 2 != 0))
    throw std::invalid_argument("Invalid plate geometry/hysteresis");
}
void Tracker::reset() { initialized_ = false; selected_ = -1; }
bool Tracker::valid(double stamp) const {
  return initialized_ && std::isfinite(stamp) && stamp >= stamp_ &&
    stamp - last_seen_ <= config_.timeout;
}
Tracker::State Tracker::state_at(double stamp) const {
  State result = state_;
  const double dt = stamp - stamp_;
  for (int i : {0, 2, 4, 6}) result[i] += dt * result[i + 1];
  result[6] = wrap(result[6]);
  return result;
}
Observation Tracker::measurement(const State & x, int index) const {
  const double a = wrap(x[6] + 2.0 * pi * index / config_.plate_count);
  const bool odd = config_.alternating_geometry && index % 2 != 0;
  const double radius = x[odd ? 9 : 8];
  return {{x[0] - radius * std::cos(a), x[2] - radius * std::sin(a),
           x[4] + (odd ? x[10] : 0.0)}, a};
}
Observation Tracker::plate_at(int index, double stamp) const {
  return measurement(state_at(stamp), index);
}
Eigen::Matrix<double, 4, 11> Tracker::jacobian(int index) const {
  Eigen::Matrix<double, 4, 11> h = Eigen::Matrix<double, 4, 11>::Zero();
  const bool odd = config_.alternating_geometry && index % 2 != 0;
  const int radius_index = odd ? 9 : 8;
  const double a = state_[6] + 2.0 * pi * index / config_.plate_count;
  h(0, 0) = h(1, 2) = h(2, 4) = h(3, 6) = 1;
  h(0, 6) = state_[radius_index] * std::sin(a); h(0, radius_index) = -std::cos(a);
  h(1, 6) = -state_[radius_index] * std::cos(a); h(1, radius_index) = -std::sin(a);
  if (odd) h(2, 10) = 1;
  return h;
}
void Tracker::predict(double stamp) {
  covariance_ = covariance_at(stamp);
  state_ = state_at(stamp);
  stamp_ = stamp;
}
Tracker::Covariance Tracker::covariance_at(double stamp) const {
  const double dt = stamp - stamp_;
  Covariance f = Covariance::Identity(), q = Covariance::Zero();
  for (int i : {0, 2, 4, 6}) {
    f(i, i + 1) = dt;
    const double variance = i == 6 ? config_.angular_acceleration_variance : config_.acceleration_variance;
    q(i, i) = variance * std::pow(dt, 4) / 4;
    q(i, i + 1) = q(i + 1, i) = variance * std::pow(dt, 3) / 2;
    q(i + 1, i + 1) = variance * dt * dt;
  }
  q(8, 8) = 1e-5 * dt;
  if (config_.alternating_geometry) q(9, 9) = q(10, 10) = 1e-5 * dt;
  return f * covariance_ * f.transpose() + q;
}
bool Tracker::correct(const Observation & o, int index) {
  const auto expected = measurement(state_, index);
  Eigen::Vector4d residual;
  residual.head<3>() = o.position - expected.position;
  residual[3] = wrap(o.yaw - expected.yaw);
  const auto h = jacobian(index);
  Eigen::Matrix4d r = Eigen::Matrix4d::Identity() * config_.position_variance;
  r(3, 3) = config_.yaw_variance;
  const Eigen::Matrix4d s = h * covariance_ * h.transpose() + r;
  const Eigen::LDLT<Eigen::Matrix4d> factor(s);
  if (factor.info() != Eigen::Success || !factor.isPositive()) return false;
  const double nis = residual.dot(factor.solve(residual));
  if (!std::isfinite(nis) || nis > config_.nis_gate) return false;
  const Eigen::Matrix<double, 11, 4> k = factor.solve(h * covariance_).transpose();
  State candidate = state_ + k * residual;
  if (!candidate.allFinite() || candidate[8] < config_.min_radius || candidate[8] > config_.max_radius ||
      candidate[9] < config_.min_radius || candidate[9] > config_.max_radius ||
      std::abs(candidate[10]) > config_.max_height_difference)
    return false;
  candidate[6] = wrap(candidate[6]);
  const Covariance a = Covariance::Identity() - k * h;
  // Joseph form preserves symmetry/positive semidefiniteness under roundoff.
  covariance_ = a * covariance_ * a.transpose() + k * r * k.transpose();
  covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
  state_ = candidate;
  return true;
}
bool Tracker::update(const std::vector<Observation> & observations, double stamp,
                     const Eigen::Vector3d & camera) {
  if (!std::isfinite(stamp) || !camera.allFinite()) return false;
  if (initialized_ && stamp <= stamp_) return false;  // Never rewind on an out-of-order frame.
  if (initialized_ && stamp - last_seen_ > config_.timeout) reset();
  std::vector<Observation> good;
  for (const auto & o : observations)
    if (o.position.allFinite() && std::isfinite(o.yaw)) good.push_back(o);
  if (!initialized_) {
    if (good.empty()) return false;
    const auto it = std::min_element(good.begin(), good.end(), [&](const auto & a, const auto & b) {
      return (a.position - camera).squaredNorm() < (b.position - camera).squaredNorm();
    });
    state_ << it->position.x() + config_.radius * std::cos(it->yaw), 0,
      it->position.y() + config_.radius * std::sin(it->yaw), 0, it->position.z(), 0,
      wrap(it->yaw), 0, config_.radius, config_.radius, 0;
    covariance_.setIdentity();
    covariance_.diagonal() << 0.1, 4, 0.1, 4, 0.1, 4, 0.2, 4, 0.02, 0.02, 0.02;
    stamp_ = last_seen_ = stamp; initialized_ = true; selected_ = 0;
    return true;
  }
  predict(stamp);
  struct Pair { double cost; size_t observation; int plate; };
  std::vector<Pair> pairs;
  for (size_t j = 0; j < good.size(); ++j) for (int i = 0; i < config_.plate_count; ++i) {
    const auto p = measurement(state_, i);
    const double d = (good[j].position - p.position).norm();
    const double a = std::abs(wrap(good[j].yaw - p.yaw));
    if (d <= config_.association_distance && a <= config_.association_yaw)
      pairs.push_back({d / config_.association_distance + a / config_.association_yaw, j, i});
  }
  std::sort(pairs.begin(), pairs.end(), [](const auto & a, const auto & b) { return a.cost < b.cost; });
  std::vector<bool> used_observation(good.size()), used_plate(config_.plate_count);
  bool accepted = false;
  for (const auto & p : pairs) {
    if (used_observation[p.observation] || used_plate[p.plate]) continue;
    if (correct(good[p.observation], p.plate)) {
      used_observation[p.observation] = used_plate[p.plate] = true;
      accepted = true;
    }
  }
  if (accepted) last_seen_ = stamp;
  return accepted;
}
int Tracker::select_nearest(double stamp, const Eigen::Vector3d & camera) {
  if (!valid(stamp) || !camera.allFinite()) return -1;
  int best = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < config_.plate_count; ++i) {
    const double d = (plate_at(i, stamp).position - camera).norm();
    if (d < distance) { best = i; distance = d; }
  }
  if (selected_ < 0 || distance + config_.switch_margin <
      (plate_at(selected_, stamp).position - camera).norm()) selected_ = best;
  return selected_;
}
}
