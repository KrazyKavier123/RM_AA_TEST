#pragma once

#include <Eigen/Dense>

#include <vector>

namespace rmv {

// 单轴 MPC 参数。状态是 [角度, 角速度]，决策量是角加速度。
// 移植自 src/rm_aa_control/src/mpc.cpp（RM_AA 的 yaw/pitch 双轴轨迹规划）。
struct MpcConfig {
    int steps{25};       // 预测步数
    int iters{300};      // 梯度迭代上限
    double dt{0.02};     // 步长 (s)
    double angle_w{40.0};  // 角度误差权重
    double rate_w{1.0};    // 角速度误差权重
    double effort_w{0.1};  // 角加速度代价权重
    double max_acc{6.0};   // 角加速度上限 (rad/s^2)
    double tol{1e-5};      // 收敛阈值
};

// 规划结果：states[k] 是未来第 k+1 步的 [角度, 角速度]，accs[k] 是对应的角加速度。
struct MpcPlan {
    bool converged{false};
    std::vector<Eigen::Vector2d> states;
    Eigen::VectorXd accs;
};

class AxisMpc {
public:
    explicit AxisMpc(MpcConfig config = {}) : cfg_(config) { build(); }

    // measured = [当前角度, 当前角速度]；reference[k] = [参考角度, 参考角速度]。
    MpcPlan solve(const Eigen::Vector2d& measured,
                  const std::vector<Eigen::Vector2d>& reference) const;

private:
    void build();

    MpcConfig cfg_;
    Eigen::MatrixXd free_;      // 无控制量时各步状态的自由响应
    Eigen::MatrixXd response_;  // 控制量到状态的响应矩阵
    Eigen::MatrixXd hessian_;
    Eigen::VectorXd weights_;
    double step_{0.01};
};

}  // namespace rmv
