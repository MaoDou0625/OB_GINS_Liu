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
    WheelNHCFactor(double t, double dt, double t0, double weight, const Eigen::Vector3d& l_sensor_odopoint) 
        : t_(t), dt_(dt), t0_(t0), weight_(weight), l_sensor_odopoint_(l_sensor_odopoint) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const p2, const T* const p3, 
                    const T* const q_body_imu_ptr,
                    const T* const l_body_sensor_ptr,
                    T* residuals) const {
        
        using SE3T = Sophus::SE3<T>;
        using Vec3T = Eigen::Matrix<T, 3, 1>;
        using QuatT = Eigen::Quaternion<T>;
        using ResT = typename spline::BSplineEvaluator::Result<T>;

        Eigen::Map<const SE3T> T0(p0);
        Eigen::Map<const SE3T> T1(p1);
        Eigen::Map<const SE3T> T2(p2);
        Eigen::Map<const SE3T> T3(p3);
        
        Eigen::Map<const QuatT> q_body_imu(q_body_imu_ptr);
        Eigen::Map<const Vec3T> l_body_sensor(l_body_sensor_ptr);

        T t_val = T(t_);
        T t_start = T(t0_) + T(dt_);
        T u = (t_val - t_start) / T(dt_);

        ResT res = spline::BSplineEvaluator::Evaluate<T>(
            u, T(dt_), 
            SE3T(T0), SE3T(T1), SE3T(T2), SE3T(T3) 
        );

        // 动态计算轮心在 Body 系下的位置
        Vec3T l_nhc = l_body_sensor + q_body_imu * l_sensor_odopoint_.cast<T>();

        // 计算轮心在 Body 系下的速度 (利用杆臂补偿公式: v_p = v_b + w_b x l_p)
        Vec3T v_nhc_b = res.v_body + res.w_body.cross(l_nhc);
        
        T weight = T(weight_);

        // 约束侧向 (Y) 和 垂向 (Z) 速度为 0
        residuals[0] = v_nhc_b.y() * weight;
        residuals[1] = v_nhc_b.z() * weight;

        return true;
    }

    static ceres::CostFunction* Create(double t, double dt, double t0, double weight, const Eigen::Vector3d& l_sensor_odopoint) {
        return new ceres::AutoDiffCostFunction<WheelNHCFactor, 2, 7, 7, 7, 7, 4, 3>(
            new WheelNHCFactor(t, dt, t0, weight, l_sensor_odopoint));
    }

private:
    double t_;
    double dt_;
    double t0_;
    double weight_;
    Eigen::Vector3d l_sensor_odopoint_;
};

} // namespace factors
} // namespace ob_gins

#endif // OB_GINS_FACTORS_WHEEL_NHC_FACTOR_H
