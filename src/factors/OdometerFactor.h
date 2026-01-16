#ifndef OB_GINS_FACTORS_ODOMETER_FACTOR_H
#define OB_GINS_FACTORS_ODOMETER_FACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {
namespace factors {

struct OdometerFactor {
    OdometerFactor(double t, double dt, double t0, double velocity) 
        : t_(t), dt_(dt), t0_(t0), velocity_(velocity) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const p2, const T* const p3, 
                    const T* const l_ecc_ptr, 
                    T* residuals) const {
        
        using SE3T = Sophus::SE3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using ResT = typename spline::BSplineEvaluator::Result<T>;

        Eigen::Map<const SE3T> T0(p0);
        Eigen::Map<const SE3T> T1(p1);
        Eigen::Map<const SE3T> T2(p2);
        Eigen::Map<const SE3T> T3(p3);
        Eigen::Map<const Vec3T> l_ecc(l_ecc_ptr);

        T t_val = T(t_);
        T t_start = T(t0_) + T(dt_);
        T u = (t_val - t_start) / T(dt_);

        ResT res = spline::BSplineEvaluator::Evaluate<T>(
            u, T(dt_), SE3T(T0), SE3T(T1), SE3T(T2), SE3T(T3)
        );

        Vec3T w_cross_l = res.w_body.cross(l_ecc);
        Vec3T v_lever_arm_w = res.pose.so3() * w_cross_l;
        Vec3T v_axle_w = res.v_world + v_lever_arm_w;
        Vec3T v_axle_b = res.pose.so3().inverse() * v_axle_w;

        // Constraint: v_x = measured velocity
        // Weighting: 10.0 (similar to NHC)
        residuals[0] = (v_axle_b.x() - T(velocity_)) * T(10.0);

        return true;
    }

    static ceres::CostFunction* Create(double t, double dt, double t0, double velocity) {
        return new ceres::AutoDiffCostFunction<OdometerFactor, 1, 7, 7, 7, 7, 3>(
            new OdometerFactor(t, dt, t0, velocity));
    }

private:
    double t_, dt_, t0_, velocity_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_ODOMETER_FACTOR_H
