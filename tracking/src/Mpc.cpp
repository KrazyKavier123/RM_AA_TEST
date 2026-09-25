#include "Mpc.hpp"

namespace rmv {

// 离散双积分模型：角度' = 角度 + 角速度*dt + 0.5*u*dt^2，角速度' = 角速度 + u*dt。
// 代价 = Σ[angle_w*角度误差² + rate_w*角速度误差²] + effort_w*Σu²，
// 用投影梯度下降求解：每步把 u 截断到 ±max_acc，梯度残差小于 tol 即认为收敛。
// 这里只预计算与参数有关的矩阵，求解时不做任何动态分配以外的额外处理。
void AxisMpc::build() {
    const int n = cfg_.steps;
    free_.resize(2 * n, 2);
    response_ = Eigen::MatrixXd::Zero(2 * n, n);
    weights_.resize(2 * n);
    for (int k = 0; k < n; ++k) {
        free_.row(2 * k) << 1.0, (k + 1) * cfg_.dt;
        free_.row(2 * k + 1) << 0.0, 1.0;
        weights_[2 * k] = cfg_.angle_w;
        weights_[2 * k + 1] = cfg_.rate_w;
        for (int j = 0; j <= k; ++j) {
            response_(2 * k, j) = (k - j + 0.5) * cfg_.dt * cfg_.dt;
            response_(2 * k + 1, j) = cfg_.dt;
        }
    }
    hessian_ = response_.transpose() * weights_.asDiagonal() * response_ +
               cfg_.effort_w * Eigen::MatrixXd::Identity(n, n);
    // 用无穷范数界估计最大特征值，保证投影梯度步长稳定。
    step_ = 1.0 / hessian_.cwiseAbs().rowwise().sum().maxCoeff();
}

MpcPlan AxisMpc::solve(const Eigen::Vector2d& measured,
                       const std::vector<Eigen::Vector2d>& reference) const {
    MpcPlan plan;
    if (reference.size() != static_cast<std::size_t>(cfg_.steps)) return plan;

    Eigen::VectorXd ref(2 * cfg_.steps);
    for (int k = 0; k < cfg_.steps; ++k) ref.segment<2>(2 * k) = reference[k];
    const Eigen::VectorXd free = free_ * measured;
    const Eigen::VectorXd gradient =
        response_.transpose() * weights_.asDiagonal() * (free - ref);

    Eigen::VectorXd u = Eigen::VectorXd::Zero(cfg_.steps);
    for (int i = 0; i < cfg_.iters; ++i) {
        const Eigen::VectorXd next = (u - step_ * (hessian_ * u + gradient))
            .cwiseMax(-cfg_.max_acc).cwiseMin(cfg_.max_acc);
        const double residual = (next - u).lpNorm<Eigen::Infinity>() / step_;
        u = next;
        if (residual < cfg_.tol) {
            plan.converged = true;
            break;
        }
    }

    const Eigen::VectorXd states = free + response_ * u;
    plan.accs = u;
    for (int k = 0; k < cfg_.steps; ++k) plan.states.emplace_back(states.segment<2>(2 * k));
    return plan;
}

}  // namespace rmv
