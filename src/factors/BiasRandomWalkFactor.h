#ifndef OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H
#define OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <cmath>

namespace ob_gins {
namespace factors {

/**
 * @brief 一阶高斯-马尔可夫过程因子 (Gauss-Markov Bias Factor)
 * 模型: b_j = exp(-dt/tau) * b_i + w,  w ~ N(0, Q)
 * 当 tau -> inf 时，退化为随机游走模型
 */
class BiasRandomWalkFactor : public ceres::SizedCostFunction<3, 3, 3> {
public:
    BiasRandomWalkFactor(double dt, double sigma, double tau) 
        : dt_(dt), sigma_(sigma), tau_(tau) {
        
        // 状态转移系数 Phi = exp(-dt / tau)
        phi_ = std::exp(-dt_ / tau_);

        // 离散化噪声协方差 Q = sigma^2 * (tau/2) * (1 - exp(-2*dt/tau))
        // 当 dt << tau 时，Q \approx sigma^2 * dt
        double q_discrete = std::pow(sigma_, 2) * (tau_ / 2.0) * (1.0 - std::exp(-2.0 * dt_ / tau_));
        double inv_std = 1.0 / std::sqrt(q_discrete);

        sqrt_info_.setIdentity();
        sqrt_info_ *= inv_std;
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        
        Eigen::Map<const Eigen::Vector3d> b_i(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> b_j(parameters[1]);
        Eigen::Map<Eigen::Vector3d> res(residuals);

        // Residual = SqrtInfo * (b_j - Phi * b_i)
        res = sqrt_info_ * (b_j - phi_ * b_i);

        if (jacobians) {
            // Jacobian w.r.t b_i: -Phi * SqrtInfo
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_i(jacobians[0]);
                J_i = -phi_ * sqrt_info_;
            }
            // Jacobian w.r.t b_j: +SqrtInfo
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_j(jacobians[1]);
                J_j = sqrt_info_;
            }
        }

        return true;
    }

    static ceres::CostFunction* Create(double dt, double sigma, double tau) {
        return new BiasRandomWalkFactor(dt, sigma, tau);
    }

private:
    double dt_;
    double sigma_;
    double tau_;
    double phi_;
    Eigen::Matrix3d sqrt_info_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H
