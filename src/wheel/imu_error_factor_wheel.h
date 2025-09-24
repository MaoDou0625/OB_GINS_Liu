/*
 * Wheel-IMU error prior factor (ceres)
 */

#ifndef WHEEL_IMU_ERROR_FACTOR_H
#define WHEEL_IMU_ERROR_FACTOR_H

#include "preintegration_wheel_base.h"

#include <ceres/ceres.h>

class WheelImuErrorFactor : public ceres::CostFunction {
public:
    explicit WheelImuErrorFactor(std::shared_ptr<WheelPreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {
        *mutable_parameter_block_sizes() = preintegration_->imuErrorNumBlocksParameters();
        set_num_residuals(preintegration_->imuErrorNumResiduals());
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        preintegration_->imuErrorEvaluate(parameters, residuals);
        if (jacobians) {
            if (jacobians[0]) {
                preintegration_->imuErrorJacobian(jacobians[0]);
            }
        }
        return true;
    }

private:
    std::shared_ptr<WheelPreintegrationBase> preintegration_;
};

#endif // WHEEL_IMU_ERROR_FACTOR_H

