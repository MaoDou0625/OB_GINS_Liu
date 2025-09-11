/*
 * Differential yaw-rate factor using left/right wheel speeds.
 * Residual model (scalar):
 *   r = (omega_meas_z - bg_z) - (v_L - v_R)/b
 * where bg_z is the z-axis gyro bias contained in the mix parameter block
 * layout [v(3), bg(3), ba(3), ...]. Units are rad/s.
 *
 * Two usage modes:
 *  - Precomputed omega_wheel: pass omega_wheel and only add mix block.
 *  - Estimate baseline b online: pass raw v_L/v_R and add an extra scalar
 *    parameter block (b). The Jacobian wrt b is (v_L - v_R)/b^2 / sigma.
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
        , mix_dim_(mix_dim)
        , use_param_b_(false)
        , vL_(0.0)
        , vR_(0.0) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(mix_dim_); // mix: v(3), bg(3), ...
    }

    // With baseline parameter: parameter blocks = [mix_dim, 1(baseline)]
    DiffYawFactor(double omega_meas_z, double vL, double vR, double sigma, int mix_dim, bool with_baseline_param)
        : omega_meas_z_(omega_meas_z)
        , omega_wheel_(0.0)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0)
        , mix_dim_(mix_dim)
        , use_param_b_(with_baseline_param)
        , vL_(vL)
        , vR_(vR) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(mix_dim_); // mix
        if (use_param_b_) mutable_parameter_block_sizes()->push_back(1); // b
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double *mix = parameters[0];
        // bg indices in mix: 3..5 (x,y,z)
        const double bg_z = (mix_dim_ >= 6) ? mix[5] : 0.0;

        double omega_wheel = omega_wheel_;
        double b = 0.0;
        if (use_param_b_) {
            b = parameters[1][0];
            const double denom = (fabs(b) > 1e-9) ? b : (b >= 0.0 ? 1e-9 : -1e-9);
            omega_wheel = (vL_ - vR_) / denom;
        }

        const double pred = (omega_meas_z_ - bg_z) - omega_wheel;
        residuals[0]      = pred * inv_sigma_;

        if (jacobians && jacobians[0]) {
            // Only d r / d bg_z is non-zero: -1 / sigma
            for (int i = 0; i < mix_dim_; ++i) jacobians[0][i] = 0.0;
            if (mix_dim_ >= 6) jacobians[0][5] = -inv_sigma_;
        }
        if (use_param_b_ && jacobians && jacobians[1]) {
            // dr/db = +(vL - vR)/b^2 / sigma (note residual sign)
            const double denom = (fabs(b) > 1e-9) ? b : (b >= 0.0 ? 1e-9 : -1e-9);
            jacobians[1][0] = ((vL_ - vR_) / (denom * denom)) * inv_sigma_;
        }

        return true;
    }

private:
    double omega_meas_z_;
    double omega_wheel_;
    double inv_sigma_;
    int mix_dim_;
    bool use_param_b_;
    double vL_;
    double vR_;
};

#endif // DIFF_YAW_FACTOR_H

