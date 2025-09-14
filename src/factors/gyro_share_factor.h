/*
 * Gyro share factor between body IMU and wheel IMU.
 * Residual (3D): r = (omega_body - bg) - R_bw(ang_base + d_ang) * omega_wheel
 * - omega_body: measured angular rate in body frame (rad/s)
 * - omega_wheel: wheel IMU measured angular rate in its own frame (rad/s)
 * - R_bw: rotation from wheel-IMU frame to body frame (using base + small delta)
 * Params:
 *   - mix: [v(3), bg(3), ba(3), ...] -> uses bg indices [3..5]
 *   - d_ang: 3 small angles (roll,pitch,yaw) to perturb base extrinsic
 */

#ifndef GYRO_SHARE_FACTOR_H
#define GYRO_SHARE_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include "src/common/rotation.h"

class GyroShareFactor : public ceres::CostFunction {
public:
    GyroShareFactor(const Eigen::Vector3d &omega_body,
                    const Eigen::Vector3d &omega_wheel,
                    const Eigen::Vector3d &rbw_base,
                    double sigma, int mix_dim)
        : omega_body_(omega_body)
        , omega_wheel_(omega_wheel)
        , rbw_base_(rbw_base)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0)
        , mix_dim_(mix_dim) {
        set_num_residuals(3);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(mix_dim_); // mix (for bg)
        mutable_parameter_block_sizes()->push_back(3);        // d_ang (delta extrinsic)
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double *mix   = parameters[0];
        const double *d_ang = parameters[1];

        const Eigen::Vector3d bg(mix_dim_ >= 6 ? mix[3] : 0.0,
                                  mix_dim_ >= 6 ? mix[4] : 0.0,
                                  mix_dim_ >= 6 ? mix[5] : 0.0);

        Eigen::Vector3d ang = rbw_base_ + Eigen::Vector3d(d_ang[0], d_ang[1], d_ang[2]);
        Eigen::Matrix3d Rbw = Rotation::euler2matrix(ang);

        Eigen::Vector3d pred = (omega_body_ - bg) - Rbw * omega_wheel_;
        Eigen::Map<Eigen::Vector3d> r(residuals);
        r = pred * inv_sigma_;

        if (jacobians) {
            if (jacobians[0]) {
                // dr/dmix: only bg columns non-zero -> -I
                Eigen::Map<Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor>> Jm(jacobians[0], 3, mix_dim_);
                Jm.setZero();
                if (mix_dim_ >= 6) {
                    Jm(0,3) = -inv_sigma_;
                    Jm(1,4) = -inv_sigma_;
                    Jm(2,5) = -inv_sigma_;
                }
            }
            if (jacobians[1]) {
                // numerical approx wrt d_ang
                const double eps = 1e-6;
                Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
                for (int k = 0; k < 3; ++k) {
                    Eigen::Vector3d dang = Eigen::Vector3d::Zero();
                    dang[k] = eps;
                    Eigen::Matrix3d R2   = Rotation::euler2matrix(rbw_base_ + Eigen::Vector3d(d_ang[0], d_ang[1], d_ang[2]) + dang);
                    Eigen::Vector3d p2   = (omega_body_ - bg) - R2 * omega_wheel_;
                    Eigen::Vector3d p1   = pred;
                    Eigen::Vector3d dp   = (p2 - p1) / eps * inv_sigma_;
                    J.col(k)             = dp;
                }
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> Ja(jacobians[1]);
                Ja = J;
            }
        }

        return true;
    }

private:
    Eigen::Vector3d omega_body_;
    Eigen::Vector3d omega_wheel_;
    Eigen::Vector3d rbw_base_;
    double inv_sigma_;
    int mix_dim_;
};

#endif // GYRO_SHARE_FACTOR_H

