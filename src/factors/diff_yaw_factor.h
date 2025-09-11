/*
 * Differential yaw-rate factor using left/right wheel speeds.
 * Residual model (scalar):
 *   r = (omega_meas_z - bg_z) - (v_L - v_R)/b
 * where bg_z is the z-axis gyro bias contained in the mix parameter block
 * layout [v(3), bg(3), ba(3), ...]. Units are rad/s.
 */

#ifndef DIFF_YAW_FACTOR_H
#define DIFF_YAW_FACTOR_H

#include <ceres/ceres.h>

class DiffYawFactor : public ceres::CostFunction {
public:
    // omega_meas_z: measured body-frame yaw rate (rad/s)
    // omega_wheel:  wheel-derived yaw rate (rad/s) = (v_L - v_R) / b
    // sigma:        standard deviation (rad/s)
    // mix_dim:      size of mix parameter block (>= 6)
    DiffYawFactor(double omega_meas_z, double omega_wheel, double sigma, int mix_dim)
        : omega_meas_z_(omega_meas_z)
        , omega_wheel_(omega_wheel)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0)
        , mix_dim_(mix_dim) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(mix_dim_); // mix: v(3), bg(3), ...
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double *mix = parameters[0];
        // bg indices in mix: 3..5 (x,y,z)
        const double bg_z = (mix_dim_ >= 6) ? mix[5] : 0.0;

        const double pred = (omega_meas_z_ - bg_z) - omega_wheel_;
        residuals[0]      = pred * inv_sigma_;

        if (jacobians && jacobians[0]) {
            // Only d r / d bg_z is non-zero: -1 / sigma
            for (int i = 0; i < mix_dim_; ++i) jacobians[0][i] = 0.0;
            if (mix_dim_ >= 6) jacobians[0][5] = -inv_sigma_;
        }

        return true;
    }

private:
    double omega_meas_z_;
    double omega_wheel_;
    double inv_sigma_;
    int mix_dim_;
};

#endif // DIFF_YAW_FACTOR_H

