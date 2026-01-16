#ifndef OB_GINS_FACTORS_WHEEL_NHC_FACTOR_H
#define OB_GINS_FACTORS_WHEEL_NHC_FACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {
namespace factors {

struct WheelNHCFactor {
    WheelNHCFactor(double t, double dt, double t0) 
        : t_(t), dt_(dt), t0_(t0) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const p2, const T* const p3, 
                    const T* const l_ecc_ptr, 
                    T* residuals) const {
        
        using SE3T = Sophus::SE3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using ResT = typename spline::BSplineEvaluator::Result<T>;

        // 1. Map parameters
        // Assume p0..p3 are standard 7-param blocks compatible with SophusSE3Manifold (Quat, Trans)
        // Wait, standard Ceres quaternion is [w, x, y, z]. Sophus is [x, y, z, w].
        // If we use SophusSE3Manifold, it expects [x, y, z, w] for Eigen Quaternion map.
        // Let's assume the parameters are stored in Sophus compatible layout.
        
        // However, if the project uses PoseManifold (global), check its layout again.
        // We previously found PoseManifold expects [Translation, Quaternion].
        // But our SophusSE3Manifold expects [Quaternion, Translation].
        // The Factor implementation must assume a specific layout for the map.
        // If we used SophusSE3Manifold in the optimization, the parameters are [Q, T].
        // So we can map them directly to SE3T.
        
        Eigen::Map<const SE3T> T0(p0);
        Eigen::Map<const SE3T> T1(p1);
        Eigen::Map<const SE3T> T2(p2);
        Eigen::Map<const SE3T> T3(p3);
        
        Eigen::Map<const Vec3T> l_ecc(l_ecc_ptr);

        // 2. Evaluate Spline
        T t_val = T(t_);
        T t_start = T(t0_) + T(dt_); // Valid range [t1, t2)
        T u = (t_val - t_start) / T(dt_);

        ResT res = spline::BSplineEvaluator::Evaluate<T>(
            u, T(dt_), 
            // We need to pass explicit copies if Evaluate signature requires it, 
            // but we updated it to take const SE3<T>&.
            // Eigen::Map behaves as reference.
            // However, previous issue with template deduction suggests explicit cast.
            SE3T(T0), SE3T(T1), SE3T(T2), SE3T(T3) 
        );

        // 3. Compute Axle Velocity in World Frame
        // v_axle_w = v_world + R_wb * (omega_body x l_ecc)
        
        Vec3T w_cross_l = res.w_body.cross(l_ecc);
        Vec3T v_lever_arm_w = res.pose.so3() * w_cross_l;
        Vec3T v_axle_w = res.v_world + v_lever_arm_w;

        // 4. Project to Body Frame
        // v_axle_b = R_wb^T * v_axle_w
        Vec3T v_axle_b = res.pose.so3().inverse() * v_axle_w;

        // 5. Residuals (NHC)
        // Y (Lateral) and Z (Vertical) velocity should be 0.
        // X is forward velocity (odometry).
        
        // Weighting? Standard deviation?
        // Let's assume a weight of 10.0 for now as requested.
        T weight = T(10.0);

        residuals[0] = v_axle_b.y() * weight;
        residuals[1] = v_axle_b.z() * weight;

        return true;
    }

    static ceres::CostFunction* Create(double t, double dt, double t0) {
        // Residuals: 2
        // Param Blocks: 
        // 0: CP0 (7)
        // 1: CP1 (7)
        // 2: CP2 (7)
        // 3: CP3 (7)
        // 4: LeverArm (3)
        return new ceres::AutoDiffCostFunction<WheelNHCFactor, 2, 7, 7, 7, 7, 3>(
            new WheelNHCFactor(t, dt, t0));
    }

private:
    double t_;
    double dt_;
    double t0_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_WHEEL_NHC_FACTOR_H
