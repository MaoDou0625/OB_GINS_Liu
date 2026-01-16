#ifndef OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H
#define OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace ob_gins {
namespace factors {

class BiasRandomWalkFactor : public ceres::SizedCostFunction<3, 3, 3> {
public:
    BiasRandomWalkFactor(double dt, double sigma) : dt_(dt), sigma_(sigma) {
        // Q = sigma^2 * dt
        // Info = Q^-1 = 1 / (sigma^2 * dt)
        // SqrtInfo = 1 / (sigma * sqrt(dt))
        
        double inv_std = 1.0 / (sigma_ * std::sqrt(dt_));
        sqrt_info_.setIdentity();
        sqrt_info_ *= inv_std;
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        
        Eigen::Map<const Eigen::Vector3d> b_i(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> b_j(parameters[1]);
        Eigen::Map<Eigen::Vector3d> res(residuals);

        // Residual = SqrtInfo * (b_j - b_i)
        res = sqrt_info_ * (b_j - b_i);

        if (jacobians) {
            // Jacobian w.r.t b_i: -SqrtInfo
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_i(jacobians[0]);
                J_i = -sqrt_info_;
            }
            // Jacobian w.r.t b_j: +SqrtInfo
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_j(jacobians[1]);
                J_j = sqrt_info_;
            }
        }

        return true;
    }

private:
    double dt_;
    double sigma_;
    Eigen::Matrix3d sqrt_info_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_BIAS_RANDOM_WALK_FACTOR_H
