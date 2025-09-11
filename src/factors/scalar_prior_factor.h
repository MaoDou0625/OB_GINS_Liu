/*
 * Simple 1D Gaussian prior: r = (x - prior) / sigma
 */

#ifndef SCALAR_PRIOR_FACTOR_H
#define SCALAR_PRIOR_FACTOR_H

#include <ceres/ceres.h>

class ScalarPriorFactor : public ceres::CostFunction {
public:
    ScalarPriorFactor(double prior, double sigma)
        : prior_(prior)
        , inv_sigma_(sigma > 0.0 ? 1.0 / sigma : 1.0) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        mutable_parameter_block_sizes()->push_back(1);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double x = parameters[0][0];
        residuals[0]   = (x - prior_) * inv_sigma_;
        if (jacobians && jacobians[0]) {
            jacobians[0][0] = inv_sigma_;
        }
        return true;
    }

private:
    double prior_;
    double inv_sigma_;
};

#endif // SCALAR_PRIOR_FACTOR_H

