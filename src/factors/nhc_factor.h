/*
 * Nonholonomic constraint (NHC) factor for wheeled vehicles
 * Softly enforces zero lateral and vertical body-frame velocity:
 *   r = [ (C_bn v)_y, (C_bn v)_z ] ~ 0
 */

#ifndef NHC_FACTOR_H
#define NHC_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

class NhcFactor : public ceres::CostFunction {

public:
    explicit NhcFactor(double sigma = 0.1, int mix_dim = 9)
        : inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0)
        , mix_dim_(mix_dim) {
        set_num_residuals(2);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(7);        // pose: p(3) + q(4)
        mutable_parameter_block_sizes()->push_back(mix_dim_); // mix: v(3) + ...
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // Pose
        const double *pose = parameters[0];
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);

        // Mix (first 3 are v in navigation frame)
        const double *mix = parameters[1];
        Eigen::Vector3d v_nav(mix[0], mix[1], mix[2]);

        Eigen::Matrix3d Cbn  = q.inverse().toRotationMatrix();
        Eigen::Vector3d v_b  = Cbn * v_nav;

        residuals[0] = v_b.y() * inv_sigma_;
        residuals[1] = v_b.z() * inv_sigma_;

        if (jacobians) {
            // d r / d pose (only attitude part is non-zero)
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_pose(jacobians[0]);
                J_pose.setZero();

                // dr/dtheta = [ (v_b x e_y)^T; (v_b x e_z)^T ]
                const Eigen::Vector3d ey = Eigen::Vector3d::UnitY();
                const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
                Eigen::Vector3d j_y     = v_b.cross(ey) * inv_sigma_;
                Eigen::Vector3d j_z     = v_b.cross(ez) * inv_sigma_;
                J_pose.block<1, 3>(0, 3) = j_y.transpose();
                J_pose.block<1, 3>(1, 3) = j_z.transpose();
            }

            // d r / d mix (first three entries are velocity in nav frame)
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> J_mix(jacobians[1], 2, mix_dim_);
                J_mix.setZero();
                Eigen::Matrix<double, 2, 3> Jv;
                Jv.row(0) = (Cbn.row(1)) * inv_sigma_;
                Jv.row(1) = (Cbn.row(2)) * inv_sigma_;
                J_mix.block<2, 3>(0, 0) = Jv;
            }
        }

        return true;
    }

private:
    double inv_sigma_;
    int mix_dim_;
};

#endif // NHC_FACTOR_H

