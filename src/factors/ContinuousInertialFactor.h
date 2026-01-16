#ifndef OB_GINS_FACTORS_CONTINUOUS_INERTIAL_FACTOR_H
#define OB_GINS_FACTORS_CONTINUOUS_INERTIAL_FACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {
namespace factors {

struct ContinuousInertialFactor {
    // Note: l_ga is now an optimization parameter block, not a fixed member
    ContinuousInertialFactor(double t_meas, const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas,
                             const Eigen::Vector3d& gravity,
                             double dt, double t0)
        : t_meas_(t_meas), accel_meas_(accel_meas), gyro_meas_(gyro_meas),
          gravity_(gravity), dt_(dt), t0_(t0) {}

    template <typename T>
    bool operator()(const T* const cp0, const T* const cp1, const T* const cp2, const T* const cp3,
                    const T* const bg0, const T* const bg1, const T* const bg2, const T* const bg3,
                    const T* const ba0, const T* const ba1, const T* const ba2, const T* const ba3,
                    const T* const l_ga_ptr, // New Parameter Block
                    T* residuals) const {
        
        using SE3T = Sophus::SE3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using ResT = typename spline::BSplineEvaluator::Result<T>;

        // 1. Map Parameters
        Eigen::Map<const SE3T> T0(cp0);
        Eigen::Map<const SE3T> T1(cp1);
        Eigen::Map<const SE3T> T2(cp2);
        Eigen::Map<const SE3T> T3(cp3);

        Eigen::Map<const Vec3T> bg1_vec(bg1);
        Eigen::Map<const Vec3T> bg2_vec(bg2);
        Eigen::Map<const Vec3T> ba1_vec(ba1);
        Eigen::Map<const Vec3T> ba2_vec(ba2);
        
        Eigen::Map<const Vec3T> l_ga(l_ga_ptr);

        (void)bg0; (void)bg3; (void)ba0; (void)ba3;

        // 2. Evaluate Spline
        T t_val = T(t_meas_);
        T t_start = T(t0_) + T(dt_);
        T u = (t_val - t_start) / T(dt_);

        ResT res = spline::BSplineEvaluator::Evaluate<T>(
            u, T(dt_), SE3T(T0), SE3T(T1), SE3T(T2), SE3T(T3)
        );

        // 3. Interpolate Bias
        Vec3T bg = bg1_vec * (T(1.0) - u) + bg2_vec * u;
        Vec3T ba = ba1_vec * (T(1.0) - u) + ba2_vec * u;

        // 4. Gyro Residual
        Vec3T gyro_pred = res.w_body + bg;
        Vec3T resid_g =  gyro_meas_.cast<T>() - gyro_pred;

        // 5. Accel Residual
        Vec3T acc_gyro = res.pose.so3().inverse() * (res.a_world - gravity_.cast<T>());

        // Lever Arm Effect (Centrifugal + Tangential)
        Vec3T w = res.w_body;
        Vec3T alpha = res.alpha_body;

        Vec3T acc_lever = alpha.cross(l_ga) + w.cross(w.cross(l_ga));
        
        Vec3T acc_total = acc_gyro + acc_lever + ba;
        
        Vec3T resid_a = accel_meas_.cast<T>() - acc_total;

        // 6. Output
        residuals[0] = resid_g[0];
        residuals[1] = resid_g[1];
        residuals[2] = resid_g[2];
        residuals[3] = resid_a[0];
        residuals[4] = resid_a[1];
        residuals[5] = resid_a[2];

        return true;
    }

    static ceres::CostFunction* Create(double t_meas, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro,
                                       const Eigen::Vector3d& g,
                                       double dt, double t0) {
        // Residuals: 6
        // Param Blocks: 
        // 4 Poses (7 each)
        // 4 Gyro Biases (3 each)
        // 4 Accel Biases (3 each)
        // 1 Lever Arm (3)
        return new ceres::AutoDiffCostFunction<ContinuousInertialFactor, 6, 
            7, 7, 7, 7,  // CP 0-3
            3, 3, 3, 3,  // Bg 0-3
            3, 3, 3, 3,  // Ba 0-3
            3            // l_ga
        >(new ContinuousInertialFactor(t_meas, accel, gyro, g, dt, t0));
    }

private:
    double t_meas_;
    Eigen::Vector3d accel_meas_;
    Eigen::Vector3d gyro_meas_;
    Eigen::Vector3d gravity_;
    double dt_;
    double t0_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_CONTINUOUS_INERTIAL_FACTOR_H