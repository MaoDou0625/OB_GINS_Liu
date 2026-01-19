#ifndef OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H
#define OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H

#include <ceres/ceres.h>
#include <ceres/autodiff_manifold.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace ob_gins {
namespace spline {

// Functor for AutoDiffManifold
struct SophusSE3Functor {
    // Plus: T_new = T_old * exp(delta)
    // Renamed to Plus as Ceres AutoDiffManifold seems to require it explicitly in this version.
    template <typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
        Eigen::Map<const Sophus::SE3<T>> T_old(x);
        Eigen::Map<const Eigen::Matrix<T, 6, 1>> delta_vec(delta);
        
        // Right perturbation (Body frame)
        Sophus::SE3<T> T_new = T_old * Sophus::SE3<T>::exp(delta_vec);
        
        Eigen::Map<Sophus::SE3<T>> T_out(x_plus_delta);
        T_out = T_new;
        return true;
    }

    // Also support operator() just in case some logic uses it.
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        return Plus(x, delta, x_plus_delta);
    }

    // Minus: delta = log(T_x^{-1} * T_y)
    template <typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const {
        Eigen::Map<const Sophus::SE3<T>> T_y(y);
        Eigen::Map<const Sophus::SE3<T>> T_x(x);
        
        // Consistent with Plus: T_y = T_x * exp(delta)
        Eigen::Matrix<T, 6, 1> delta = (T_x.inverse() * T_y).log();
        
        Eigen::Map<Eigen::Matrix<T, 6, 1>> delta_out(y_minus_x);
        delta_out = delta;
        return true;
    }
};

using SophusSE3Manifold = ceres::AutoDiffManifold<SophusSE3Functor, 7, 6>;

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H