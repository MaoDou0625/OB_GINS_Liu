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
                             const Eigen::Vector3d& gravity, const Eigen::Vector3d& omega_ie,
                             double dt, double t0, double sigma_a, double sigma_g)
        : t_meas_(t_meas), accel_meas_(accel_meas), gyro_meas_(gyro_meas),
          gravity_(gravity), omega_ie_(omega_ie), dt_(dt), t0_(t0), sigma_a_(sigma_a), sigma_g_(sigma_g) {}

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
        // w_meas = R_L^b * (w_ie_L) + w_Lb_b + bias
        // w_pred = res.w_body + bg + R_b^L^T * w_ie_L
        Vec3T w_ie_b = res.pose.so3().inverse() * omega_ie_.cast<T>();
        Vec3T gyro_pred = res.w_body + bg + w_ie_b;
        Vec3T resid_g =  gyro_meas_.cast<T>() - gyro_pred;

        // 5. Accel Residual
        // f_b = R_L^b * (a_L - g_L + 2 * w_ie_L x v_L)
        // Note: omega_ie is constant in L frame (if L is earth-fixed)
        Vec3T coriolis = T(2.0) * omega_ie_.cast<T>().cross(res.v_world);
        Vec3T acc_inertial = res.a_world - gravity_.cast<T>() + coriolis;
        
        Vec3T acc_gyro = res.pose.so3().inverse() * acc_inertial;

        // Lever Arm Effect (Centrifugal + Tangential)
        Vec3T w = res.w_body;
        Vec3T alpha = res.alpha_body;

        Vec3T acc_lever = alpha.cross(l_ga) + w.cross(w.cross(l_ga));
        
        Vec3T acc_total = acc_gyro + acc_lever + ba;
        
        Vec3T resid_a = accel_meas_.cast<T>() - acc_total;

        // 6. Output
        T inv_sigma_g = T(1.0 / sigma_g_);
        T inv_sigma_a = T(1.0 / sigma_a_);

        residuals[0] = resid_g[0] * inv_sigma_g;
        residuals[1] = resid_g[1] * inv_sigma_g;
        residuals[2] = resid_g[2] * inv_sigma_g;
        residuals[3] = resid_a[0] * inv_sigma_a;
        residuals[4] = resid_a[1] * inv_sigma_a;
        residuals[5] = resid_a[2] * inv_sigma_a;

        return true;
    }

    static ceres::CostFunction* Create(double t_meas, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro,
                                       const Eigen::Vector3d& g, const Eigen::Vector3d& omega_ie,
                                       double dt, double t0, double sigma_a, double sigma_g) {
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
        >(new ContinuousInertialFactor(t_meas, accel, gyro, g, omega_ie, dt, t0, sigma_a, sigma_g));
    }

private:
    double t_meas_;
    Eigen::Vector3d accel_meas_;
    Eigen::Vector3d gyro_meas_;
    Eigen::Vector3d gravity_;
    Eigen::Vector3d omega_ie_;
    double dt_;
    double t0_;
    double sigma_a_;
    double sigma_g_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_CONTINUOUS_INERTIAL_FACTOR_H