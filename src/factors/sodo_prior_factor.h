/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 * (small prior on odometer scale)
 */

#ifndef SODO_PRIOR_FACTOR_H
#define SODO_PRIOR_FACTOR_H

#include <ceres/ceres.h>

class SodoPriorFactor : public ceres::CostFunction {
public:
    SodoPriorFactor(double prior = 1.0, double sigma = 2.0e4 * 1.0e-6)
        : prior_(prior)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(1);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double sodo = parameters[0][0];
        residuals[0]      = (sodo - prior_) * inv_sigma_;
        if (jacobians && jacobians[0]) {
            jacobians[0][0] = inv_sigma_;
        }
        return true;
    }

private:
    double prior_;
    double inv_sigma_;
};

#endif // SODO_PRIOR_FACTOR_H
