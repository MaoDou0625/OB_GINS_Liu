/* Wheel-IMU Preintegration Normal (no odo, no earth rotation) */

#ifndef WHEEL_PREINTEGRATION_NORMAL_H
#define WHEEL_PREINTEGRATION_NORMAL_H

#include "preintegration_wheel_base.h"

class WheelPreintegrationNormal : public WheelPreintegrationBase {

public:
    WheelPreintegrationNormal(std::shared_ptr<WheelIntegrationParameters> parameters, const IMU &imu0,
                              WheelIntegrationState state);

    Eigen::MatrixXd evaluate(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                             double *residuals) override;

    Eigen::MatrixXd residualJacobianPose0(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianPose1(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix0(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                                         double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix1(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                                         double *jacobian) override;
    int numResiduals() override;
    std::vector<int> numBlocksParameters() override;

    static WheelIntegrationStateData stateToData(const WheelIntegrationState &state);
    static WheelIntegrationState stateFromData(const WheelIntegrationStateData &data);
    void constructState(const double *const *parameters, WheelIntegrationState &state0,
                        WheelIntegrationState &state1) override;

    int imuErrorNumResiduals() override;
    std::vector<int> imuErrorNumBlocksParameters() override;
    void imuErrorEvaluate(const double *const *parameters, double *residuals) override;
    void imuErrorJacobian(double *jacobian) override;

protected:
    void integrationProcess(unsigned long index) override;
    void resetState(const WheelIntegrationState &state) override;

    void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) override;

private:
    void resetState(const WheelIntegrationState &state, int num);
    void setNoiseMatrix();

public:
    static constexpr int NUM_MIX = 9;

private:
    static constexpr int NUM_STATE = 15;
    static constexpr int NUM_NOISE = 12;
};

#endif // WHEEL_PREINTEGRATION_NORMAL_H

