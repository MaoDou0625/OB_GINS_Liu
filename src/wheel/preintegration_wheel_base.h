/*
 * Wheel-IMU Preintegration Base
 * Copied from src/preintegration/preintegration_base.h with renamed types
 * and independent state/parameter definitions.
 */

#ifndef WHEEL_PREINTEGRATION_BASE_H
#define WHEEL_PREINTEGRATION_BASE_H

#include "src/common/rotation.h"
#include "src/common/types.h"

#include "integration_state_wheel.h"

#include <memory>
#include <vector>

class WheelPreintegrationBase {

public:
    WheelPreintegrationBase(std::shared_ptr<WheelIntegrationParameters> parameters, const IMU &imu0,
                            WheelIntegrationState state);

    virtual ~WheelPreintegrationBase() = default;

    const WheelIntegrationState &currentState() { return current_state_; }
    const WheelIntegrationState &deltaState() { return delta_state_; }

    double deltaTime() const { return delta_time_; }
    double startTime() const { return start_time_; }
    double endTime() const { return end_time_; }

    const Vector3d &gravity() { return gravity_; }
    const std::vector<IMU> &imuBuffer() { return imu_buffer_; }

    void addNewImu(const IMU &imu);
    void reintegration(WheelIntegrationState &state);

public:
    virtual Eigen::MatrixXd evaluate(const WheelIntegrationState &state0, const WheelIntegrationState &state1,
                                     double *residuals) = 0;

    virtual Eigen::MatrixXd residualJacobianPose0(const WheelIntegrationState &state0,
                                                  const WheelIntegrationState &state1, double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianPose1(const WheelIntegrationState &state0,
                                                  const WheelIntegrationState &state1, double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianMix0(const WheelIntegrationState &state0,
                                                 const WheelIntegrationState &state1, double *jacobian)  = 0;
    virtual Eigen::MatrixXd residualJacobianMix1(const WheelIntegrationState &state0,
                                                 const WheelIntegrationState &state1, double *jacobian)  = 0;

    virtual int numResiduals()                     = 0;
    virtual std::vector<int> numBlocksParameters() = 0;

    virtual void constructState(const double *const *parameters, WheelIntegrationState &state0,
                                WheelIntegrationState &state1) = 0;

    virtual int imuErrorNumResiduals()                     = 0;
    virtual std::vector<int> imuErrorNumBlocksParameters() = 0;

    virtual void imuErrorEvaluate(const double *const *parameters, double *residuals) = 0;

    virtual void imuErrorJacobian(double *jacobian) = 0;

protected:
    // need compensate bias
    virtual void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) = 0;

    virtual void resetState(const WheelIntegrationState &state) = 0;
    virtual void integrationProcess(unsigned long index)        = 0;

    static void stateToData(const WheelIntegrationState &state, WheelIntegrationStateData &data);
    static void stateFromData(const WheelIntegrationStateData &data, WheelIntegrationState &state);

    void integration(const IMU &imu_pre, const IMU &imu_cur);

    IMU compensationBias(const IMU &imu) const;
    IMU compensationScale(const IMU &imu) const;

public:
    static constexpr int NUM_POSE = 7;

protected:
    // TODO: You can set these parameters according to your IMU
    static constexpr double IMU_GRY_BIAS_STD = 7200 / 3600.0 * M_PI / 180.0; // 7200 deg / hr
    static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;               // 20000 mGal
    static constexpr double IMU_SCALE_STD    = 5.0e3 * 1.0e-6;               // 5000 PPM
    static constexpr double IMU_ACC_Z_SCALE  = 100;
    static constexpr double ODO_SCALE_STD    = 2.0e4 * 1.0e-6; // 0.02

    const std::shared_ptr<WheelIntegrationParameters> parameters_;

    WheelIntegrationState current_state_;
    WheelIntegrationState delta_state_;

    std::vector<IMU> imu_buffer_;
    double delta_time_{0};
    double start_time_;
    double end_time_;

    Vector3d gravity_;

    Eigen::MatrixXd jacobian_, covariance_;
    Eigen::MatrixXd noise_;
    Eigen::MatrixXd sqrt_information_;

    Quaterniond corrected_q_;
    Vector3d corrected_p_, corrected_v_;
};

#endif // WHEEL_PREINTEGRATION_BASE_H

