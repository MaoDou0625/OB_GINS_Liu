#ifndef OB_GINS_SPLINE_SPLINE_POSE_FACTOR_H
#define OB_GINS_SPLINE_SPLINE_POSE_FACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {
namespace spline {

struct SplinePoseFactor {
    SplinePoseFactor(double t_meas, const Sophus::SE3d& T_meas, double dt, double t0) 
        : t_meas_(t_meas), T_meas_(T_meas), dt_(dt), t0_(t0) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const p2, const T* const p3, T* residuals) const {
        
        // Map parameters to Sophus::SE3<T>
        // We assume p0 points to 7 doubles (Quaternion + Translation)
        Eigen::Map<const Sophus::SE3<T>> T0(p0);
        Eigen::Map<const Sophus::SE3<T>> T1(p1);
        Eigen::Map<const Sophus::SE3<T>> T2(p2);
        Eigen::Map<const Sophus::SE3<T>> T3(p3);
        
        // Calculate normalized time u
        // Assuming t0 is the timestamp of T0.
        // The active segment for cubic B-spline with T0..T3 is typically [t1, t2) = [t0+dt, t0+2dt).
        // u = (t - t1) / dt
        
        T t_val = T(t_meas_);
        T t_start = T(t0_) + T(dt_);
        T u = (t_val - t_start) / T(dt_);

        // Evaluate Pose
        // Assign to local variables to ensure type matching for EvaluatePose
        Sophus::SE3<T> val_T0 = T0;
        Sophus::SE3<T> val_T1 = T1;
        Sophus::SE3<T> val_T2 = T2;
        Sophus::SE3<T> val_T3 = T3;

        Sophus::SE3<T> T_spline = BSplineEvaluator::EvaluatePose(u, val_T0, val_T1, val_T2, val_T3);
        
        // Residual: log(T_meas^-1 * T_spline)
        Sophus::SE3<T> T_meas_T = T_meas_.cast<T>();
        Sophus::Vector6<T> error = (T_meas_T.inverse() * T_spline).log();
        
        // Weighting could be applied here (e.g. sqrt(info)), but we return raw residual.
        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
        res = error;
        
        return true;
    }

    static ceres::CostFunction* Create(double t_meas, const Sophus::SE3d& T_meas, double dt, double t0) {
        return new ceres::AutoDiffCostFunction<SplinePoseFactor, 6, 7, 7, 7, 7>(
            new SplinePoseFactor(t_meas, T_meas, dt, t0));
    }

private:
    double t_meas_;
    Sophus::SE3d T_meas_;
    double dt_;
    double t0_;
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_SPLINE_POSE_FACTOR_H
