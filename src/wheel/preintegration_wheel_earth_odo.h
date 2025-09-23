/* Wheel-IMU Preintegration with Earth rotation + odometer */

#ifndef WHEEL_PREINTEGRATION_EARTH_ODO_H
#define WHEEL_PREINTEGRATION_EARTH_ODO_H

#include "preintegration_wheel_base.h"

class WheelPreintegrationEarthOdo : public WheelPreintegrationBase {
public:
    WheelPreintegrationEarthOdo(std::shared_ptr<WheelIntegrationParameters> parameters, const IMU &imu0,
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
    static constexpr int NUM_MIX = 12;

private:
    static constexpr int NUM_STATE = 21;
    static constexpr int NUM_NOISE = 18;

    static constexpr int NUM_ERROR_RESIDUAL = 7;

    Matrix3d cvb_;
    Vector3d lodo_;

    Vector3d corrected_s_;

    // Earth-rotation related buffers
    Quaterniond q0_;
    Vector3d iewn_;
    Matrix3d iewn_skew_;
    std::vector<std::pair<double, Vector3d>> pn_;
    Vector3d dpn_, dvn_;
    Quaterniond qb0b1_;
};

#endif // WHEEL_PREINTEGRATION_EARTH_ODO_H
