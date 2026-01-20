#ifndef OB_GINS_FACTORS_PRIOR_FACTORS_H
#define OB_GINS_FACTORS_PRIOR_FACTORS_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ob_gins {
namespace factors {

/**
 * @brief 杆臂先验因子，对 3D 向量添加高斯先验约束
 */
struct LeverArmPriorFactor {
    LeverArmPriorFactor(const Eigen::Vector3d& prior, double sigma)
        : prior_(prior), sigma_(sigma) {}

    template <typename T>
    bool operator()(const T* const l_ptr, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> l(l_ptr);
        
        residuals[0] = (l[0] - T(prior_[0])) / T(sigma_);
        residuals[1] = (l[1] - T(prior_[1])) / T(sigma_);
        residuals[2] = (l[2] - T(prior_[2])) / T(sigma_);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& prior, double sigma) {
        return new ceres::AutoDiffCostFunction<LeverArmPriorFactor, 3, 3>(
            new LeverArmPriorFactor(prior, sigma));
    }

    Eigen::Vector3d prior_;
    double sigma_;
};

/**
 * @brief 旋转先验因子，对四元数添加高斯先验约束（切空间误差模型）
 */
struct RotationPriorFactor {
    RotationPriorFactor(const Eigen::Quaterniond& prior_q, double sigma)
        : prior_q_inv_(prior_q.inverse()), sigma_(sigma) {}

    template <typename T>
    bool operator()(const T* const q_ptr, T* residuals) const {
        Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);
        
        // 计算误差四元数: q_error = q_current * q_prior_inverse
        Eigen::Quaternion<T> q_error = q * prior_q_inv_.template cast<T>();
        
        // 使用小角近似 (Small angle approximation) 将误差映射到切空间
        // delta_vec = 2 * [q_error.x, q_error.y, q_error.z]
        residuals[0] = T(2.0) * q_error.x() / T(sigma_);
        residuals[1] = T(2.0) * q_error.y() / T(sigma_);
        residuals[2] = T(2.0) * q_error.z() / T(sigma_);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& prior_q, double sigma) {
        return new ceres::AutoDiffCostFunction<RotationPriorFactor, 3, 4>(
            new RotationPriorFactor(prior_q, sigma));
    }

    Eigen::Quaterniond prior_q_inv_;
    double sigma_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_PRIOR_FACTORS_H