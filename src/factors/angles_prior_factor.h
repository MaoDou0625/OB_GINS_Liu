/*
 * Small L2 prior on 3D angle vector (roll, pitch, yaw)
 */

#ifndef ANGLES_PRIOR_FACTOR_H
#define ANGLES_PRIOR_FACTOR_H

#include <ceres/ceres.h>

class AnglesPriorFactor : public ceres::CostFunction {
public:
    // sigma in radians (per-axis). Default ~3 deg.
    explicit AnglesPriorFactor(double sigma_rad = 0.05235987755982988)
        : inv_sigma_(sigma_rad > 0.0 ? 1.0 / sigma_rad : 1.0) {
        set_num_residuals(3);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double *ang = parameters[0];
        residuals[0]      = ang[0] * inv_sigma_;
        residuals[1]      = ang[1] * inv_sigma_;
        residuals[2]      = ang[2] * inv_sigma_;
        if (jacobians && jacobians[0]) {
            double *J = jacobians[0];
            // Row-major 3x3 identity scaled
            J[0] = inv_sigma_; J[1] = 0;          J[2] = 0;
            J[3] = 0;          J[4] = inv_sigma_; J[5] = 0;
            J[6] = 0;          J[7] = 0;          J[8] = inv_sigma_;
        }
        return true;
    }

private:
    double inv_sigma_;
};

#endif // ANGLES_PRIOR_FACTOR_H
