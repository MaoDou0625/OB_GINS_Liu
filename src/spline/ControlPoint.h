#ifndef OB_GINS_SPLINE_CONTROL_POINT_H
#define OB_GINS_SPLINE_CONTROL_POINT_H

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ob_gins {
namespace spline {

class ControlPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlPoint() : timestamp_(0.0) {
        bg_.setZero();
        ba_.setZero();
    }

    ControlPoint(double t, const Sophus::SE3d& pose) 
        : timestamp_(t), pose_(pose) {
        bg_.setZero();
        ba_.setZero();
    }

    // Raw data access for Ceres (parameter blocks)
    // Sophus::SE3d data layout: quaternion (4) + translation (3) usually, 
    // but typically we pass the underlying pointer.
    // Note: Sophus::SE3d::data() returns double* since reasonable versions.
    double* pose_data() { return pose_.data(); }
    const double* pose_data() const { return pose_.data(); }

    double* bg_data() { return bg_.data(); }
    const double* bg_data() const { return bg_.data(); }

    double* ba_data() { return ba_.data(); }
    const double* ba_data() const { return ba_.data(); }

    // Object access
    Sophus::SE3d& pose() { return pose_; }
    const Sophus::SE3d& pose() const { return pose_; }

    double& timestamp() { return timestamp_; }
    const double& timestamp() const { return timestamp_; }
    
    Eigen::Vector3d& bg() { return bg_; }
    const Eigen::Vector3d& bg() const { return bg_; }

    Eigen::Vector3d& ba() { return ba_; }
    const Eigen::Vector3d& ba() const { return ba_; }

private:
    double timestamp_;
    Sophus::SE3d pose_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d ba_;
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_CONTROL_POINT_H
