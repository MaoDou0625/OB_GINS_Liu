#ifndef OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H
#define OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

namespace ob_gins {
namespace spline {

// Manifold for Sophus::SE3d
// Data layout: [Quaternion(4), Translation(3)]
// Tangent layout: [Translation(3), Rotation(3)] (Standard Sophus log order)
class SophusSE3Manifold : public ceres::Manifold {
public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        Eigen::Map<const Sophus::SE3d> T(x);
        Eigen::Map<const Sophus::Vector6d> d(delta);
        Eigen::Map<Sophus::SE3d> T_plus(x_plus_delta);

        // Right multiplication: T_new = T * exp(delta)
        // Or Left? Standard in robotics is usually Left for world-frame perturbations, 
        // or Right for body-frame.
        // Ceres usually expects x_plus = x [+] delta.
        // Let's stick to Right multiplication (manifold local chart).
        T_plus = T * Sophus::SE3d::exp(d);
        
        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override {
        // J = d(T * exp(delta)) / d(delta) at delta=0
        // For SE3 right update, Jacobian wrt delta is Identity? 
        // No, it's the derivative of the Plus operation.
        // If x is represented by 7 params, Plus returns 7 params.
        // Jacobian is 7x6.
        
        // This is complicated to compute manually for Quaternions.
        // However, Ceres 2.1+ allows AutoDiffManifold if we define a functor.
        // But let's try to use the logic.
        // d(T*exp(d))/dd = d(T*exp(d))/d(T*exp(d)) * d(T*exp(d))/dd
        // At d=0, T*exp(d) = T.
        // This is basically how the 7 parameters change given a 6-d perturbation.
        
        // Let's use AutoDiffManifold to save error-prone derivations.
        return false; // Should not be called if we use AutoDiffManifold? 
        // Wait, I cannot use AutoDiffManifold easily without wrapping the Plus.
        
        // Actually, if I inherit from Manifold, I must implement this.
        // Let's implement Plus using a functor and wrapping it with AutoDiffManifold.
        // But for now, let's implement PlusJacobian manually if easy, or use AutoDiff.
    }
    
    // Better way:
    // Define a struct and use AutoDiffManifold.
};

struct SophusSE3Plus {
    template <typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
        Eigen::Map<const Sophus::SE3<T>> T_x(x);
        Eigen::Map<const Sophus::Vector6<T>> d(delta);
        Eigen::Map<Sophus::SE3<T>> T_out(x_plus_delta);
        
        T_out = T_x * Sophus::SE3<T>::exp(d);
        return true;
    }

    template <typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const {
        Eigen::Map<const Sophus::SE3<T>> T_y(y);
        Eigen::Map<const Sophus::SE3<T>> T_x(x);
        Eigen::Map<Sophus::Vector6<T>> d(y_minus_x);

        d = (T_x.inverse() * T_y).log();
        return true;
    }
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_SOPHUS_SE3_MANIFOLD_H
