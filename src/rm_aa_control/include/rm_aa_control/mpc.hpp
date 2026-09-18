#pragma once
#include <Eigen/Dense>
#include <vector>

namespace rm_aa_control {
struct MpcConfig {
  int horizon{25}, iterations{300};
  double dt{0.02}, angle_weight{40}, velocity_weight{1}, effort_weight{0.1};
  double max_acceleration{6}, tolerance{1e-5};
};
struct AxisPlan {
  bool converged{false};
  std::vector<Eigen::Vector2d> states;  // angle [rad], rate [rad/s], starts at k=1
  Eigen::VectorXd accelerations;
};
class AxisMpc {
public:
  explicit AxisMpc(MpcConfig config = {});
  AxisPlan solve(const Eigen::Vector2d & measured,
                 const std::vector<Eigen::Vector2d> & reference) const;
private:
  MpcConfig config_;
  Eigen::MatrixXd response_, free_, hessian_;
  Eigen::VectorXd weights_;
  double step_;
};
}
