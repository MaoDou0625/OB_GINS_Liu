#ifndef OB_GINS_SPLINE_CONTROL_POINT_H
#define OB_GINS_SPLINE_CONTROL_POINT_H

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ob_gins {
namespace spline {

class ControlPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlPoint() : timestamp_(0.0) {}

    ControlPoint(double t, const Sophus::SE3d& pose) 
        : timestamp_(t), pose_(pose) {}

    // Raw data access for Ceres (parameter blocks)
    // Sophus::SE3d data layout: quaternion (4) + translation (3) usually, 
    // but typically we pass the underlying pointer.
    // Note: Sophus::SE3d::data() returns double* since reasonable versions.
    double* pose_data() { return pose_.data(); }
    const double* pose_data() const { return pose_.data(); }

    // Object access
    Sophus::SE3d& pose() { return pose_; }
    const Sophus::SE3d& pose() const { return pose_; }

    double& timestamp() { return timestamp_; }
    const double& timestamp() const { return timestamp_; }
    
private:
    double timestamp_;
    Sophus::SE3d pose_;
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_CONTROL_POINT_H
