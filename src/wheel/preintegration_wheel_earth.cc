/* Wheel-IMU Preintegration Earth implementation */

#include "preintegration_wheel_earth.h"
#include "src/common/earth.h"

WheelPreintegrationEarth::WheelPreintegrationEarth(std::shared_ptr<WheelIntegrationParameters> parameters,
                                                   const IMU &imu0, WheelIntegrationState state)
    : WheelPreintegrationBase(std::move(parameters), imu0, std::move(state)) {

    // Reset state
    resetState(current_state_, NUM_STATE);

    // Set initial noise matrix
    setNoiseMatrix();
}

Eigen::MatrixXd WheelPreintegrationEarth::evaluate(const WheelIntegrationState &state0,
                                                   const WheelIntegrationState &state1, double *residuals) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(covariance_.inverse()).matrixL().transpose();

    Eigen::Map<Eigen::Matrix<double, NUM_STATE, 1>> residual(residuals);

    Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg = jacobian_.block<3, 3>(6, 9);

    // bias deltas
    Vector3d dbg = state0.bg - delta_state_.bg;
    Vector3d dba = state0.ba - delta_state_.ba;

    // position correction from earth rotation
    Vector3d p_cor{0, 0, 0};
    for (const auto &pn : pn_) {
        p_cor += (pn.second - state0.p) * pn.first;
    }
    p_cor = 2.0 * iewn_skew_ * p_cor;

    // velocity correction
    Vector3d v_cor = 2.0 * iewn_skew_ * (state1.p - state0.p);

    // attitude correction
    Vector3d dnn    = -iewn_ * delta_time_;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

    dpn_ = state1.p - state0.p - state0.v * delta_time_ - 0.5 * gravity_ * delta_time_ * delta_time_ + p_cor;
    dvn_ = state1.v - state0.v - gravity_ * delta_time_ + v_cor;

    // corrected integration
    corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
    corrected_v_ = delta_state_.v + dv_dba * dba + dv_dbg * dbg;
    corrected_q_ = delta_state_.q * Rotation::rotvec2quaternion(dq_dbg * dbg);

    Quaterniond qnb0 = state0.q.inverse();
    Matrix3d cnb0    = qnb0.toRotationMatrix();
    qb0b1_           = state1.q.inverse() * qnn * state0.q;

    // residuals
    residual.block<3, 1>(0, 0)  = cnb0 * dpn_ - corrected_p_;
    residual.block<3, 1>(3, 0)  = cnb0 * dvn_ - corrected_v_;
    residual.block<3, 1>(6, 0)  = 2 * (qb0b1_ * corrected_q_).vec();
    residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
    residual.block<3, 1>(12, 0) = state1.ba - state0.ba;

    residual = sqrt_information_ * residual;
    return residual;
}

Eigen::MatrixXd WheelPreintegrationEarth::residualJacobianPose0(const WheelIntegrationState &state0,
                                                                const WheelIntegrationState &state1,
                                                                double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Quaterniond qnb0 = state0.q.inverse();
    Matrix3d cnb0    = qnb0.toRotationMatrix();

    jaco.block(0, 0, 3, 3) = -cnb0 - 2.0 * cnb0 * iewn_skew_ * delta_time_;
    jaco.block(0, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dpn_);
    jaco.block(3, 0, 3, 3) = -2.0 * cnb0 * iewn_skew_;
    jaco.block(3, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dvn_);
    jaco.block(6, 3, 3, 3) =
        (Rotation::quaternionleft(qb0b1_) * Rotation::quaternionright(corrected_q_)).bottomRightCorner<3, 3>();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarth::residualJacobianPose1(const WheelIntegrationState &state0,
                                                                const WheelIntegrationState &state1,
                                                                double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();

    jaco.block(0, 0, 3, 3) = cnb0;
    jaco.block(3, 0, 3, 3) = 2.0 * cnb0 * iewn_skew_;
    jaco.block(6, 3, 3, 3) = -Rotation::quaternionright(qb0b1_ * corrected_q_).bottomRightCorner<3, 3>();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarth::residualJacobianMix0(const WheelIntegrationState &state0,
                                                               const WheelIntegrationState &state1,
                                                               double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(0, 12);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(3, 9);
    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(3, 12);
    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(6, 9);

    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();

    jaco.block(0, 0, 3, 3) = -cnb0 * delta_time_;
    jaco.block(0, 3, 3, 3) = -dp_dbg;
    jaco.block(0, 6, 3, 3) = -dp_dba;
    jaco.block(3, 0, 3, 3) = -cnb0;
    jaco.block(3, 3, 3, 3) = -dv_dbg;
    jaco.block(3, 6, 3, 3) = -dv_dba;
    jaco.block(6, 3, 3, 3) =
        (Rotation::quaternionleft(qb0b1_ * delta_state_.q).bottomRightCorner<3, 3>()) * dq_dbg;
    jaco.block(9, 3, 3, 3)  = -Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = -Eigen::Matrix3d::Identity();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarth::residualJacobianMix1(const WheelIntegrationState &state0,
                                                               const WheelIntegrationState &state1,
                                                               double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(3, 0, 3, 3)  = state0.q.inverse().toRotationMatrix();
    jaco.block(9, 3, 3, 3)  = Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = Eigen::Matrix3d::Identity();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

int WheelPreintegrationEarth::numResiduals() { 
    return NUM_STATE; 
}

std::vector<int> WheelPreintegrationEarth::numBlocksParameters() { 
    return {NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX}; 
}

WheelIntegrationStateData WheelPreintegrationEarth::stateToData(const WheelIntegrationState &state) {
    WheelIntegrationStateData data{};
    WheelPreintegrationBase::stateToData(state, data);
    return data;
}

WheelIntegrationState WheelPreintegrationEarth::stateFromData(const WheelIntegrationStateData &data) {
    WheelIntegrationState state{};
    WheelPreintegrationBase::stateFromData(data, state);
    return state;
}

void WheelPreintegrationEarth::constructState(const double *const *parameters, WheelIntegrationState &state0,
                                              WheelIntegrationState &state1) {
    state0 = WheelIntegrationState{
        .p  = {parameters[0][0], parameters[0][1], parameters[0][2]},
        .q  = {parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]},
        .v  = {parameters[1][0], parameters[1][1], parameters[1][2]},
        .bg = {parameters[1][3], parameters[1][4], parameters[1][5]},
        .ba = {parameters[1][6], parameters[1][7], parameters[1][8]},
    };

    state1 = WheelIntegrationState{
        .p  = {parameters[2][0], parameters[2][1], parameters[2][2]},
        .q  = {parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]},
        .v  = {parameters[3][0], parameters[3][1], parameters[3][2]},
        .bg = {parameters[3][3], parameters[3][4], parameters[3][5]},
        .ba = {parameters[3][6], parameters[3][7], parameters[3][8]},
    };
}

void WheelPreintegrationEarth::integrationProcess(unsigned long index) {
    IMU imu_pre = compensationBias(imu_buffer_[index - 1]);
    IMU imu_cur = compensationBias(imu_buffer_[index]);

    // time accumulation
    double dt = imu_cur.dt;
    delta_time_ += dt;

    end_time_           = imu_cur.time;
    current_state_.time = imu_cur.time;

    // position/velocity integration
    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    Vector3d dv_cor_g = (gravity_ - 2.0 * iewn_.cross(current_state_.v)) * dt;

    // earth rotation compensation
    Vector3d dnn    = -iewn_ * dt;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

    Vector3d dvel =
        0.5 * (Matrix3d::Identity() + qnn.toRotationMatrix()) * current_state_.q.toRotationMatrix() * dvfb + dv_cor_g;

    // average velocity for position
    current_state_.p += dt * current_state_.v + 0.5 * dt * dvel;
    current_state_.v += dvel;

    pn_.emplace_back(std::make_pair(dt, current_state_.p));

    // attitude
    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);
    current_state_.q = qnn * current_state_.q * Rotation::rotvec2quaternion(dtheta);
    current_state_.q.normalize();

    // preintegration delta
    dnn  = -(delta_time_ - 0.5 * dt) * iewn_;
    dvel = (q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix() * dvfb;

    delta_state_.p += dt * delta_state_.v + 0.5 * dt * dvel;
    delta_state_.v += dvel;
    delta_state_.q *= Rotation::rotvec2quaternion(dtheta);
    delta_state_.q.normalize();

    updateJacobianAndCovariance(imu_pre, imu_cur);
}

void WheelPreintegrationEarth::resetState(const WheelIntegrationState &state) {
    resetState(state, NUM_STATE);
}

void WheelPreintegrationEarth::updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) {
    // dp, dv, dq, dbg, dba
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE);

    double dt = imu_cur.dt;

    Vector3d dnn  = -iewn_ * delta_time_;
    Matrix3d cbb0 = -(q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix();

    // jacobian: phi = I + F * dt
    phi.block<3, 3>(0, 0)   = Matrix3d::Identity();
    phi.block<3, 3>(0, 3)   = Matrix3d::Identity() * dt;
    phi.block<3, 3>(3, 3)   = Matrix3d::Identity();
    phi.block<3, 3>(3, 6)   = cbb0 * Rotation::skewSymmetric(imu_cur.dvel);
    phi.block<3, 3>(3, 12)  = cbb0 * dt;
    phi.block<3, 3>(6, 6)   = Matrix3d::Identity() - Rotation::skewSymmetric(imu_cur.dtheta);
    phi.block<3, 3>(6, 9)   = -Matrix3d::Identity() * dt;
    phi.block<3, 3>(9, 9)   = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);
    phi.block<3, 3>(12, 12) = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);

    jacobian_ = phi * jacobian_;

    // covariance
    Eigen::MatrixXd gt = Eigen::MatrixXd::Zero(NUM_STATE, NUM_NOISE);
    gt.block<3, 3>(3, 3)  = cbb0;
    gt.block<3, 3>(6, 0)  = -Matrix3d::Identity();
    gt.block<3, 3>(9, 6)  = Matrix3d::Identity();
    gt.block<3, 3>(12, 9) = Matrix3d::Identity();

    Eigen::MatrixXd Qk =
        0.5 * dt * (phi * gt * noise_ * gt.transpose() + gt * noise_ * gt.transpose() * phi.transpose());
    covariance_ = phi * covariance_ * phi.transpose() + Qk;
}

void WheelPreintegrationEarth::resetState(const WheelIntegrationState &state, int num) {
    delta_time_ = 0;
    delta_state_.p.setZero();
    delta_state_.q.setIdentity();
    delta_state_.v.setZero();
    delta_state_.bg = state.bg;
    delta_state_.ba = state.ba;

    jacobian_.setIdentity(num, num);
    covariance_.setZero(num, num);

    // absolute attitude at start of preintegration
    q0_ = current_state_.q;

    // earth rotation, approximate with initial position
    iewn_      = Earth::iewn(parameters_->station, current_state_.p);
    iewn_skew_ = Rotation::skewSymmetric(iewn_);

    pn_.clear();
}

void WheelPreintegrationEarth::setNoiseMatrix() {
    noise_.setIdentity(NUM_NOISE, NUM_NOISE);
    noise_.block<3, 3>(0, 0) *= parameters_->gyr_arw * parameters_->gyr_arw; // nw
    noise_.block<3, 3>(3, 3) *= parameters_->acc_vrw * parameters_->acc_vrw; // na
    noise_.block<3, 3>(6, 6) *=
        2 * parameters_->gyr_bias_std * parameters_->gyr_bias_std / parameters_->corr_time; // nbg
    noise_.block<3, 3>(9, 9) *=
        2 * parameters_->acc_bias_std * parameters_->acc_bias_std / parameters_->corr_time; // nba
}

int WheelPreintegrationEarth::imuErrorNumResiduals() { return 6; }

std::vector<int> WheelPreintegrationEarth::imuErrorNumBlocksParameters() { return std::vector<int>{NUM_MIX}; }

void WheelPreintegrationEarth::imuErrorEvaluate(const double *const *parameters, double *residuals) {
    residuals[0] = parameters[0][3] / IMU_GRY_BIAS_STD;
    residuals[1] = parameters[0][4] / IMU_GRY_BIAS_STD;
    residuals[2] = parameters[0][5] / IMU_GRY_BIAS_STD;
    residuals[3] = parameters[0][6] / IMU_ACC_BIAS_STD;
    residuals[4] = parameters[0][7] / IMU_ACC_BIAS_STD;
    residuals[5] = parameters[0][8] / IMU_ACC_BIAS_STD;
}

void WheelPreintegrationEarth::imuErrorJacobian(double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, 6, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    jaco(0, 3) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(1, 4) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(2, 5) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(3, 6) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(4, 7) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(5, 8) = 1.0 / IMU_ACC_BIAS_STD;
}
