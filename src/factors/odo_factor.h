/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ODO_FACTOR_H
#define ODO_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/rotation.h"

// Odometer (wheel speed) factor implementing
// r = t_b^T ( C_bn * v_nav + omega_b × l_b ) - s_odo * v_meas
// Parameter blocks:
//  - pose: [p(3), q(4)] with quaternion stored as (x,y,z,w) in indices [3..6]
//  - mix : [v(3), bg(3), ba(3), ...] — only v used
//  - sodo: scalar scale parameter
class OdoFactor : public ceres::CostFunction {

public:
    OdoFactor(double v_meas, double sigma, const Eigen::Vector3d &omega_cross_l,
              int mix_dim, const Eigen::Vector3d &abv_base)
        : v_meas_(v_meas)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0)
        , omega_cross_l_(omega_cross_l)
        , mix_dim_(mix_dim)
        , abv_base_(abv_base) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(7);         // pose
        mutable_parameter_block_sizes()->push_back(mix_dim_);  // mix
        mutable_parameter_block_sizes()->push_back(1);         // sodo
        mutable_parameter_block_sizes()->push_back(3);         // delta abv (roll, pitch, yaw)
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // Pose: p (unused), q as (x,y,z,w) at indices [3..6]
        const double *pose = parameters[0];
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);

        // Mix: v_nav at [0..2]
        const double *mix = parameters[1];
        Eigen::Vector3d v_nav(mix[0], mix[1], mix[2]);

        // sodo
        const double sodo = parameters[2][0];
        // delta abv (angles added to base abv)
        Eigen::Vector3d d_abv(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Matrix3d cbv = Rotation::euler2matrix(abv_base_ + d_abv).transpose();
        Eigen::Vector3d t_b = cbv * Eigen::Vector3d::UnitX();

        // Predicted tangential speed
        Eigen::Vector3d v_body = q.inverse() * v_nav;               // C_bn * v
        double v_pred          = t_b.dot(v_body + omega_cross_l_);

        residuals[0] = (v_pred - sodo * v_meas_) * inv_sigma_;

        if (jacobians) {
            // Jacobian w.r.t pose [p(3), q(4)]: only attitude columns [3..5]
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_pose(jacobians[0]);
                J_pose.setZero();
                // d(C_bn v)/d(theta) ≈ -skew(C_bn v) * dtheta, so dr/dtheta = t_b^T * (-skew(v_body))
                Eigen::Vector3d jtheta = v_body.cross(t_b); // equals -t_b x v_body
                J_pose(0, 3) = jtheta.x() * inv_sigma_;
                J_pose(0, 4) = jtheta.y() * inv_sigma_;
                J_pose(0, 5) = jtheta.z() * inv_sigma_;
            }
            // Jacobian w.r.t mix [v(3), ...]
            if (jacobians[1]) {
                Eigen::Matrix3d Cbn = q.inverse().toRotationMatrix();
                Eigen::RowVector3d jv = t_b.transpose() * Cbn; // t_b^T * C_bn
                Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>> J_mix(jacobians[1], 1, mix_dim_);
                J_mix.setZero();
                J_mix(0, 0) = jv(0) * inv_sigma_;
                J_mix(0, 1) = jv(1) * inv_sigma_;
                J_mix(0, 2) = jv(2) * inv_sigma_;
            }
            // Jacobian w.r.t sodo
            if (jacobians[2]) {
                jacobians[2][0] = -v_meas_ * inv_sigma_;
            }
            // Jacobian w.r.t delta abv (numerical approx)
            if (jacobians[3]) {
                Eigen::Vector3d v_sum = v_body + omega_cross_l_;
                const double eps      = 1e-6;
                Eigen::Vector3d Jd;
                for (int k = 0; k < 3; ++k) {
                    Eigen::Vector3d d = Eigen::Vector3d::Zero();
                    d[k]              = eps;
                    Eigen::Matrix3d cbv2 = Rotation::euler2matrix(abv_base_ + d_abv + d).transpose();
                    Eigen::Vector3d t_b2  = cbv2 * Eigen::Vector3d::UnitX();
                    double val2            = t_b2.dot(v_sum);
                    double val1            = t_b.dot(v_sum);
                    Jd[k]                  = (val2 - val1) / eps * inv_sigma_;
                }
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_abv(jacobians[3]);
                J_abv(0, 0) = Jd[0];
                J_abv(0, 1) = Jd[1];
                J_abv(0, 2) = Jd[2];
            }
        }

        return true;
    }

private:
    double v_meas_;
    double inv_sigma_;
    Eigen::Vector3d omega_cross_l_;
    int mix_dim_;
    Eigen::Vector3d abv_base_;
};

#endif // ODO_FACTOR_H
