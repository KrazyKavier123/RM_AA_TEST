#include "rm_aa_control/mpc.hpp"
#include <cmath>
#include <stdexcept>

namespace rm_aa_control {
AxisMpc::AxisMpc(MpcConfig config) : config_(config) {
  for (double v : {config.dt, config.angle_weight, config.velocity_weight,
       config.effort_weight, config.max_acceleration, config.tolerance})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid MPC parameter");
  if (config.horizon < 2 || config.horizon > 200 || config.iterations < 1 || config.iterations > 10000)
    throw std::invalid_argument("Invalid MPC dimensions/iteration limit");
  const int n = config.horizon;
  response_ = Eigen::MatrixXd::Zero(2 * n, n);
  free_.resize(2 * n, 2); weights_.resize(2 * n);
  for (int k = 0; k < n; ++k) {
    free_.row(2 * k) << 1, (k + 1) * config.dt;
    free_.row(2 * k + 1) << 0, 1;
    weights_[2 * k] = config.angle_weight; weights_[2 * k + 1] = config.velocity_weight;
    for (int j = 0; j <= k; ++j) {
      response_(2 * k, j) = (k - j + 0.5) * config.dt * config.dt;
      response_(2 * k + 1, j) = config.dt;
    }
  }
  hessian_ = response_.transpose() * weights_.asDiagonal() * response_ +
    config.effort_weight * Eigen::MatrixXd::Identity(n, n);
  // Infinity norm bounds the spectral radius: stable projected-gradient step.
  step_ = 1.0 / hessian_.cwiseAbs().rowwise().sum().maxCoeff();
}
AxisPlan AxisMpc::solve(const Eigen::Vector2d & measured,
                       const std::vector<Eigen::Vector2d> & reference) const {
  AxisPlan result;
  if (!measured.allFinite() || reference.size() != static_cast<size_t>(config_.horizon)) return result;
  Eigen::VectorXd ref(2 * config_.horizon);
  for (int k = 0; k < config_.horizon; ++k) {
    if (!reference[k].allFinite()) return result;
    ref.segment<2>(2 * k) = reference[k];
  }
  const Eigen::VectorXd free = free_ * measured;
  const Eigen::VectorXd gradient = response_.transpose() * weights_.asDiagonal() * (free - ref);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(config_.horizon);
  for (int i = 0; i < config_.iterations; ++i) {
    const Eigen::VectorXd next = (u - step_ * (hessian_ * u + gradient))
      .cwiseMax(-config_.max_acceleration).cwiseMin(config_.max_acceleration);
    const double residual = (next - u).lpNorm<Eigen::Infinity>() / step_;
    u = next;
    if (residual < config_.tolerance) { result.converged = true; break; }
  }
  const Eigen::VectorXd states = free + response_ * u;
  if (!states.allFinite() || !u.allFinite()) { result.converged = false; return result; }
  result.accelerations = u;
  for (int k = 0; k < config_.horizon; ++k) result.states.emplace_back(states.segment<2>(2 * k));
  return result;
}
}
