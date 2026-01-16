#ifndef OB_GINS_FACTORS_CONTINUOUS_GNSS_FACTOR_H
#define OB_GINS_FACTORS_CONTINUOUS_GNSS_FACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {
namespace factors {

struct ContinuousGnssFactor {
    ContinuousGnssFactor(double t, double dt, double t0, const Eigen::Vector3d& pos_meas, const Eigen::Matrix3d& sqrt_info) 
        : t_(t), dt_(dt), t0_(t0), pos_meas_(pos_meas), sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const p2, const T* const p3, 
                    const T* const l_ecc_ptr, 
                    T* residuals) const {
        
        using SE3T = Sophus::SE3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using ResT = typename spline::BSplineEvaluator::Result<T>;

        // 1. Map parameters
        // Sophus::SE3 memory layout is [qx, qy, qz, qw, tx, ty, tz]
        Eigen::Map<const SE3T> T0(p0);
        Eigen::Map<const SE3T> T1(p1);
        Eigen::Map<const SE3T> T2(p2);
        Eigen::Map<const SE3T> T3(p3);
        
        Eigen::Map<const Vec3T> l_ecc(l_ecc_ptr);

        // 2. Evaluate Spline
        T t_val = T(t_);
        T t_start = T(t0_) + T(dt_); // Valid range [t1, t2)
        T u = (t_val - t_start) / T(dt_);

        // Evaluate B-Spline at t_meas_ to get P_wb (IMU Pos) and R_wb (IMU Rot)
        // We assume the control points are strictly SE3 elements.
        ResT res = spline::BSplineEvaluator::Evaluate<T>(
            u, T(dt_), 
            SE3T(T0), SE3T(T1), SE3T(T2), SE3T(T3) 
        );

        // 3. Compute Axle Position in World Frame
        // P_axle_w = P_wb + R_wb * l_ecc
        // (Note: We apply rotation here because l_ecc moves with the wheel - it is fixed in IMU/Wheel frame)
        Vec3T P_wb = res.pose.translation();
        Vec3T P_axle_w = P_wb + res.pose.so3() * l_ecc;

        // 4. Compute Residual
        // r = P_axle_w - pos_meas
        Vec3T diff = P_axle_w - pos_meas_.cast<T>();

        // 5. Apply weight
        Eigen::Map<Vec3T> residual_map(residuals);
        residual_map = sqrt_info_.cast<T>() * diff;

        return true;
    }

    static ceres::CostFunction* Create(double t, double dt, double t0, const Eigen::Vector3d& pos_meas, const Eigen::Matrix3d& sqrt_info) {
        // Residuals: 3 (XYZ)
        // Param Blocks: 
        // 0: CP0 (7) - Pose
        // 1: CP1 (7) - Pose
        // 2: CP2 (7) - Pose
        // 3: CP3 (7) - Pose
        // 4: l_ecc (3) - Vector3
        return new ceres::AutoDiffCostFunction<ContinuousGnssFactor, 3, 7, 7, 7, 7, 3>(
            new ContinuousGnssFactor(t, dt, t0, pos_meas, sqrt_info));
    }

private:
    double t_;
    double dt_;
    double t0_;
    Eigen::Vector3d pos_meas_;
    Eigen::Matrix3d sqrt_info_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_CONTINUOUS_GNSS_FACTOR_H
