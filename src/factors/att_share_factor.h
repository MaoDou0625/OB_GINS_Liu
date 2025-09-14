/*
 * Attitude share factor between body IMU and wheel IMU.
 * Uses a measured wheel-IMU orientation (from integrating wheel dtheta)
 * as a pseudo-measurement and constrains body orientation via extrinsics.
 * Residual (3D): r = Log( q_meas * (R_wb(base+delta) * q_body)^-1 ) / sigma
 * Parameter blocks:
 *  - pose: [p(3), q(4)] (use q only)
 *  - d_ang: 3 (small angles added to base extrinsic rbw_base)
 */

#ifndef ATT_SHARE_FACTOR_H
#define ATT_SHARE_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include "src/common/rotation.h"

class AttShareFactor : public ceres::CostFunction {
public:
    AttShareFactor(const Eigen::Quaterniond &q_meas,
                   const Eigen::Vector3d &rbw_base,
                   double sigma_rad)
        : q_meas_(q_meas)
        , rbw_base_(rbw_base)
        , inv_sigma_(sigma_rad > 0.0 ? 1.0 / sigma_rad : 1.0) {
        set_num_residuals(3);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(7); // pose
        mutable_parameter_block_sizes()->push_back(3); // d_ang
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double *pose  = parameters[0];
        const double *d_ang = parameters[1];

        Eigen::Quaterniond q_body(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Vector3d ang = rbw_base_ + Eigen::Vector3d(d_ang[0], d_ang[1], d_ang[2]);
        Eigen::Matrix3d Rbw = Rotation::euler2matrix(ang);
        Eigen::Matrix3d Rwb = Rbw.transpose();

        Eigen::Matrix3d R_pred = Rwb * q_body.toRotationMatrix();
        Eigen::Quaterniond q_pred = Rotation::matrix2quaternion(R_pred);

        // q_err = q_meas * q_pred^{-1}
        Eigen::Quaterniond q_err = q_meas_ * q_pred.conjugate();
        Eigen::Vector3d r = Rotation::quaternion2vector(q_err) * inv_sigma_;
        residuals[0] = r.x(); residuals[1] = r.y(); residuals[2] = r.z();

        if (jacobians) {
            if (jacobians[0]) {
                // Approximate Jacobian w.r.t body attitude via small-angle: dr/dtheta_body ≈ -I / sigma
                Eigen::Map<Eigen::Matrix<double,3,7,Eigen::RowMajor>> Jp(jacobians[0]);
                Jp.setZero();
                Jp.block<3,3>(0,3) = -Eigen::Matrix3d::Identity() * inv_sigma_;
            }
            if (jacobians[1]) {
                // Numerical Jacobian w.r.t d_ang
                const double eps = 1e-6;
                Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
                for (int k=0;k<3;++k){
                    Eigen::Vector3d da = Eigen::Vector3d::Zero(); da[k]=eps;
                    Eigen::Matrix3d Rbw2 = Rotation::euler2matrix(rbw_base_ + Eigen::Vector3d(d_ang[0], d_ang[1], d_ang[2]) + da);
                    Eigen::Matrix3d Rwb2 = Rbw2.transpose();
                    Eigen::Quaterniond q_pred2 = Rotation::matrix2quaternion(Rwb2 * q_body.toRotationMatrix());
                    Eigen::Quaterniond q_err2 = q_meas_ * q_pred2.conjugate();
                    Eigen::Vector3d r2 = Rotation::quaternion2vector(q_err2) * inv_sigma_;
                    Eigen::Vector3d dr = (r2 - r) / eps;
                    J.col(k) = dr;
                }
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> Ja(jacobians[1]);
                Ja = J;
            }
        }

        return true;
    }

private:
    Eigen::Quaterniond q_meas_;
    Eigen::Vector3d rbw_base_;
    double inv_sigma_;
};

#endif // ATT_SHARE_FACTOR_H

