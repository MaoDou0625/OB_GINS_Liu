/* Wheel-IMU Preintegration Normal implementation */

#include "preintegration_wheel_normal.h"

#include "src/common/logging.h"

WheelPreintegrationNormal::WheelPreintegrationNormal(std::shared_ptr<WheelIntegrationParameters> parameters,
                                                     const IMU &imu0, WheelIntegrationState state)
    : WheelPreintegrationBase(std::move(parameters), imu0, std::move(state)) {

    // Reset state
    resetState(current_state_, NUM_STATE);

    // Set initial noise matrix
    setNoiseMatrix();
}

Eigen::MatrixXd WheelPreintegrationNormal::evaluate(const WheelIntegrationState &state0,
                                                    const WheelIntegrationState &state1, double *residuals) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(covariance_.inverse()).matrixL().transpose();

    Eigen::Map<Eigen::Matrix<double, NUM_STATE, 1>> residual(residuals);

    Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg = jacobian_.block<3, 3>(6, 9);

    // Bias
    Vector3d dbg = state0.bg - delta_state_.bg;
    Vector3d dba = state0.ba - delta_state_.ba;

    // Correct integration
    corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
    corrected_v_ = delta_state_.v + dv_dba * dba + dv_dbg * dbg;
    corrected_q_ = delta_state_.q * Rotation::rotvec2quaternion(dq_dbg * dbg);

    // Residuals
    residual.block<3, 1>(0, 0) = state0.q.inverse() * (state1.p - state0.p - state0.v * delta_time_ -
                                                       0.5 * gravity_ * delta_time_ * delta_time_) -
                                 corrected_p_;
    residual.block<3, 1>(3, 0)  = state0.q.inverse() * (state1.v - state0.v - gravity_ * delta_time_) - corrected_v_;
    residual.block<3, 1>(6, 0)  = 2 * (corrected_q_.inverse() * state0.q.inverse() * state1.q).vec();
    residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
    residual.block<3, 1>(12, 0) = state1.ba - state0.ba;

    residual = sqrt_information_ * residual;
    return residual;
}

Eigen::MatrixXd WheelPreintegrationNormal::residualJacobianPose0(const WheelIntegrationState &state0,
                                                                 const WheelIntegrationState &state1,
                                                                 double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(0, 0, 3, 3) = -state0.q.inverse().toRotationMatrix();
    jaco.block(0, 3, 3, 3) =
        Rotation::skewSymmetric(state0.q.inverse() * (state1.p - state0.p - state0.v * delta_time_ -
                                                      0.5 * gravity_ * delta_time_ * delta_time_));
    jaco.block(3, 3, 3, 3) =
        Rotation::skewSymmetric(state0.q.inverse() * (state1.v - state0.v - gravity_ * delta_time_));
    jaco.block(6, 3, 3, 3) =
        -(Rotation::quaternionleft(state1.q.inverse() * state0.q) * Rotation::quaternionright(corrected_q_))
             .bottomRightCorner<3, 3>();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationNormal::residualJacobianPose1(const WheelIntegrationState &state0,
                                                                 const WheelIntegrationState &state1,
                                                                 double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(0, 0, 3, 3) = state0.q.inverse().toRotationMatrix();
    jaco.block(6, 3, 3, 3) =
        Rotation::quaternionleft(corrected_q_.inverse() * state0.q.inverse() * state1.q).bottomRightCorner<3, 3>();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationNormal::residualJacobianMix0(const WheelIntegrationState &state0,
                                                                const WheelIntegrationState &state1,
                                                                double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(0, 12);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(3, 9);
    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(3, 12);
    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(6, 9);

    jaco.block(0, 0, 3, 3) = -state0.q.inverse().toRotationMatrix() * delta_time_;
    jaco.block(0, 3, 3, 3) = -dp_dbg;
    jaco.block(0, 6, 3, 3) = -dp_dba;
    jaco.block(3, 0, 3, 3) = -state0.q.inverse().toRotationMatrix();
    jaco.block(3, 3, 3, 3) = -dv_dbg;
    jaco.block(3, 6, 3, 3) = -dv_dba;
    jaco.block(6, 3, 3, 3) = -dq_dbg;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationNormal::residualJacobianMix1(const WheelIntegrationState &state0,
                                                                const WheelIntegrationState &state1,
                                                                double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(9, 0, 3, 3)   = Eigen::Matrix3d::Identity();
    jaco.block(12, 3, 3, 3)  = Eigen::Matrix3d::Identity();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

int WheelPreintegrationNormal::numResiduals() {
    return NUM_STATE;
}

std::vector<int> WheelPreintegrationNormal::numBlocksParameters() {
    return {NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX};
}

WheelIntegrationStateData WheelPreintegrationNormal::stateToData(const WheelIntegrationState &state) {
    WheelIntegrationStateData data;
    WheelPreintegrationBase::stateToData(state, data);
    return data;
}

WheelIntegrationState WheelPreintegrationNormal::stateFromData(const WheelIntegrationStateData &data) {
    WheelIntegrationState state{};
    WheelPreintegrationBase::stateFromData(data, state);
    return state;
}

void WheelPreintegrationNormal::constructState(const double *const *parameters, WheelIntegrationState &state0,
                                               WheelIntegrationState &state1) {
    // pose0 + mix0 + pose1 + mix1
    WheelIntegrationStateData data0, data1;
    memcpy(data0.pose, parameters[0], sizeof(double) * NUM_POSE);
    memcpy(data0.mix, parameters[1], sizeof(double) * NUM_MIX);
    memcpy(data1.pose, parameters[2], sizeof(double) * NUM_POSE);
    memcpy(data1.mix, parameters[3], sizeof(double) * NUM_MIX);

    state0 = stateFromData(data0);
    state1 = stateFromData(data1);
}

int WheelPreintegrationNormal::imuErrorNumResiduals() {
    return 6;
}

std::vector<int> WheelPreintegrationNormal::imuErrorNumBlocksParameters() {
    return {3, 3};
}

void WheelPreintegrationNormal::imuErrorEvaluate(const double *const *parameters, double *residuals) {
    // gyr bias + acc bias
    Eigen::Map<const Eigen::Vector3d> bg(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> ba(parameters[1]);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) = bg;
    residual.block<3, 1>(3, 0) = ba;

    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_information.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * IMU_GRY_BIAS_STD;
    sqrt_information.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * IMU_ACC_BIAS_STD;
    residual = sqrt_information * residual;
}

void WheelPreintegrationNormal::imuErrorJacobian(double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    jaco.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * IMU_GRY_BIAS_STD;
    jaco.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * IMU_ACC_BIAS_STD;
}

void WheelPreintegrationNormal::integrationProcess(unsigned long index) {
    const auto &imu_pre = imu_buffer_[index - 1];
    const auto &imu_cur = imu_buffer_[index];

    auto imu_pre_calib = compensationBias(imu_pre);
    auto imu_cur_calib = compensationBias(imu_cur);

    integration(imu_pre_calib, imu_cur_calib);

    updateJacobianAndCovariance(imu_pre_calib, imu_cur_calib);
}

void WheelPreintegrationNormal::resetState(const WheelIntegrationState &state) {
    resetState(state, NUM_STATE);
}

void WheelPreintegrationNormal::updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) {
    // two-sample correction for angular velocity
    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);

    Quaterniond dq = Rotation::rotvec2quaternion(dtheta);
    Matrix3d r     = Rotation::quaternion2matrix(dq);

    Matrix3d dp_dbg, dp_dba, dv_dbg, dv_dba;
    Matrix3d dq_dbg;

    dp_dbg = jacobian_.block<3, 3>(0, 9);
    dp_dba = jacobian_.block<3, 3>(0, 12);
    dv_dbg = jacobian_.block<3, 3>(3, 9);
    dv_dba = jacobian_.block<3, 3>(3, 12);
    dq_dbg = jacobian_.block<3, 3>(6, 9);

    // Jacobian
    jacobian_.block<3, 3>(0, 0)   = Matrix3d::Identity();
    jacobian_.block<3, 3>(0, 3)   = Matrix3d::Identity() * imu_cur.dt;
    jacobian_.block<3, 3>(0, 6)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(0, 9)   = dp_dbg - 0.5 * Rotation::skewSymmetric(imu_cur.dvel) * imu_cur.dt -
                                   1.0 / 12.0 * Rotation::skewSymmetric(imu_pre.dvel) * imu_cur.dt +
                                   1.0 / 12.0 * Rotation::skewSymmetric(imu_cur.dvel) * imu_pre.dt;
    jacobian_.block<3, 3>(0, 12)  = dp_dba + 0.5 * Matrix3d::Identity() * imu_cur.dt +
                                   1.0 / 12.0 * r * Matrix3d::Identity() * imu_pre.dt -
                                   1.0 / 12.0 * Matrix3d::Identity() * imu_cur.dt;
    jacobian_.block<3, 3>(3, 0)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(3, 3)   = Matrix3d::Identity();
    jacobian_.block<3, 3>(3, 6)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(3, 9)   = dv_dbg - Rotation::skewSymmetric(imu_cur.dvel) +
                                   1.0 / 12.0 * Rotation::skewSymmetric(imu_pre.dvel);
    jacobian_.block<3, 3>(3, 12)  = dv_dba + Matrix3d::Identity();
    jacobian_.block<3, 3>(6, 0)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(6, 3)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(6, 6)   = r.transpose();
    jacobian_.block<3, 3>(6, 9)   = dq_dbg + Matrix3d::Identity() * imu_cur.dt;
    jacobian_.block<3, 3>(6, 12)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(9, 0)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(9, 3)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(9, 6)   = Matrix3d::Zero();
    jacobian_.block<3, 3>(9, 9)   = Matrix3d::Identity();
    jacobian_.block<3, 3>(9, 12)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(12, 0)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(12, 3)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(12, 6)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(12, 9)  = Matrix3d::Zero();
    jacobian_.block<3, 3>(12, 12) = Matrix3d::Identity();

    // Covariance
    covariance_ = jacobian_ * covariance_ * jacobian_.transpose() + noise_;
}

void WheelPreintegrationNormal::resetState(const WheelIntegrationState &state, int num) {
    // delta state
    memset(&delta_state_, 0, sizeof(WheelIntegrationState));
    delta_state_.q = Quaterniond(1, 0, 0, 0);
    delta_state_.p = Vector3d::Zero();
    delta_state_.v = Vector3d::Zero();
    delta_state_.bg.setZero();
    delta_state_.ba.setZero();

    // jac-cov-noise
    jacobian_   = Eigen::MatrixXd::Identity(num, num);
    covariance_ = Eigen::MatrixXd::Zero(num, num);
    noise_      = Eigen::MatrixXd::Zero(num, num);

    // gravity
    gravity_ = Vector3d(0, 0, parameters_->gravity);
}

void WheelPreintegrationNormal::setNoiseMatrix() {
    double vrw = parameters_->acc_vrw;
    double arw = parameters_->gyr_arw;

    double dt = 1.0; // will be scaled each step in updateJacobianAndCovariance

    noise_.setZero(NUM_STATE, NUM_STATE);
    noise_.block<3, 3>(0, 0)   = Eigen::Matrix3d::Identity() * vrw * vrw * dt * dt;
    noise_.block<3, 3>(3, 3)   = Eigen::Matrix3d::Identity() * vrw * vrw * dt;
    noise_.block<3, 3>(6, 6)   = Eigen::Matrix3d::Identity() * arw * arw * dt;
    noise_.block<3, 3>(9, 9)   = Eigen::Matrix3d::Identity() * IMU_GRY_BIAS_STD * IMU_GRY_BIAS_STD * dt;
    noise_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * IMU_ACC_BIAS_STD * IMU_ACC_BIAS_STD * dt;
}

