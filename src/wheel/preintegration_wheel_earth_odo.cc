/* Wheel-IMU Preintegration Earth+Odo implementation */

#include "preintegration_wheel_earth_odo.h"
#include "src/common/earth.h"

WheelPreintegrationEarthOdo::WheelPreintegrationEarthOdo(std::shared_ptr<WheelIntegrationParameters> parameters,
                                                         const IMU &imu0, WheelIntegrationState state)
    : WheelPreintegrationBase(std::move(parameters), imu0, std::move(state)) {

    resetState(current_state_, NUM_STATE);
    setNoiseMatrix();

    cvb_  = Rotation::euler2matrix(parameters_->abv).transpose();
    lodo_ = parameters_->lodo;
}

Eigen::MatrixXd WheelPreintegrationEarthOdo::evaluate(const WheelIntegrationState &state0,
                                                      const WheelIntegrationState &state1, double *residuals) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(covariance_.inverse()).matrixL().transpose();

    Eigen::Map<Eigen::Matrix<double, NUM_STATE, 1>> residual(residuals);

    Matrix3d dp_dbg   = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba   = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg   = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba   = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg   = jacobian_.block<3, 3>(6, 9);
    Vector3d ds_dsodo = jacobian_.block<3, 1>(15, 18);
    Matrix3d ds_dbg   = jacobian_.block<3, 3>(15, 9);

    Vector3d dbg = state0.bg - delta_state_.bg;
    Vector3d dba = state0.ba - delta_state_.ba;
    double dsodo = state0.sodo - delta_state_.sodo;

    // earth corrections
    Vector3d p_cor{0, 0, 0};
    for (const auto &pn : pn_) p_cor += (pn.second - state0.p) * pn.first;
    p_cor = 2.0 * iewn_skew_ * p_cor;
    Vector3d v_cor = 2.0 * iewn_skew_ * (state1.p - state0.p);
    Vector3d dnn   = -iewn_ * delta_time_;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

    dpn_ = state1.p - state0.p - state0.v * delta_time_ - 0.5 * gravity_ * delta_time_ * delta_time_ + p_cor;
    dvn_ = state1.v - state0.v - gravity_ * delta_time_ + v_cor;

    corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
    corrected_v_ = delta_state_.v + dv_dba * dba + dv_dbg * dbg;
    corrected_q_ = delta_state_.q * Rotation::rotvec2quaternion(dq_dbg * dbg);
    corrected_s_ = delta_state_.s + ds_dbg * dbg + ds_dsodo * dsodo;

    Quaterniond qnb0 = state0.q.inverse();
    Matrix3d cnb0    = qnb0.toRotationMatrix();
    qb0b1_           = state1.q.inverse() * qnn * state0.q;

    residual.block<3, 1>(0, 0)  = cnb0 * dpn_ - corrected_p_;
    residual.block<3, 1>(3, 0)  = cnb0 * dvn_ - corrected_v_;
    residual.block<3, 1>(6, 0)  = 2 * (qb0b1_ * corrected_q_).vec();
    residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
    residual.block<3, 1>(12, 0) = state1.ba - state0.ba;
    residual.block<3, 1>(15, 0) = cnb0 * (state1.p - state0.p) - corrected_s_;
    residual(18)                = state1.sodo - state0.sodo;

    residual = sqrt_information_ * residual;
    return residual;
}

Eigen::MatrixXd WheelPreintegrationEarthOdo::residualJacobianPose0(const WheelIntegrationState &state0,
                                                                   const WheelIntegrationState &state1,
                                                                   double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();
    jaco.block(0, 0, 3, 3) = -cnb0 - 2.0 * cnb0 * iewn_skew_ * delta_time_;
    jaco.block(0, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dpn_);
    jaco.block(3, 0, 3, 3) = -2.0 * cnb0 * iewn_skew_;
    jaco.block(3, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dvn_);
    jaco.block(6, 3, 3, 3) =
        (Rotation::quaternionleft(qb0b1_) * Rotation::quaternionright(corrected_q_)).bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = -cnb0;
    jaco.block(15, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * (state1.p - state0.p));
    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarthOdo::residualJacobianPose1(const WheelIntegrationState &state0,
                                                                   const WheelIntegrationState &state1,
                                                                   double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();
    jaco.block(0, 0, 3, 3) = cnb0;
    jaco.block(3, 0, 3, 3) = 2.0 * cnb0 * iewn_skew_;
    jaco.block(6, 3, 3, 3) = -Rotation::quaternionright(qb0b1_ * corrected_q_).bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = cnb0;
    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarthOdo::residualJacobianMix0(const WheelIntegrationState &state0,
                                                                  const WheelIntegrationState &state1,
                                                                  double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(0, 12);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(3, 9);
    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(3, 12);
    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(6, 9);
    Matrix3d cnb0          = state0.q.inverse().toRotationMatrix();
    jaco.block(0, 0, 3, 3)  = -cnb0 * delta_time_;
    jaco.block(0, 3, 3, 3)  = -dp_dbg;
    jaco.block(0, 6, 3, 3)  = -dp_dba;
    jaco.block(3, 0, 3, 3)  = -cnb0;
    jaco.block(3, 3, 3, 3)  = -dv_dbg;
    jaco.block(3, 6, 3, 3)  = -dv_dba;
    jaco.block(6, 3, 3, 3)  = -dq_dbg;
    jaco.block(15, 3, 3, 3) = -jacobian_.block<3, 3>(15, 9);
    jaco.block(15, 9, 3, 1) = -jacobian_.block<3, 1>(15, 18);
    jaco.block(18, 9, 1, 1) = Eigen::Matrix<double, 1, 1>::Identity();
    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd WheelPreintegrationEarthOdo::residualJacobianMix1(const WheelIntegrationState &state0,
                                                                  const WheelIntegrationState &state1,
                                                                  double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    jaco.block(9, 0, 3, 3)  = Eigen::Matrix3d::Identity();
    jaco.block(12, 3, 3, 3) = Eigen::Matrix3d::Identity();
    jaco.block(18, 9, 1, 1) = Eigen::Matrix<double, 1, 1>::Identity();
    jaco = sqrt_information_ * jaco;
    return jaco;
}

int WheelPreintegrationEarthOdo::numResiduals() { return NUM_STATE; }
std::vector<int> WheelPreintegrationEarthOdo::numBlocksParameters() { return {NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX}; }

WheelIntegrationStateData WheelPreintegrationEarthOdo::stateToData(const WheelIntegrationState &state) {
    WheelIntegrationStateData data{};
    WheelPreintegrationBase::stateToData(state, data);
    data.mix[9] = state.sodo;
    return data;
}

WheelIntegrationState WheelPreintegrationEarthOdo::stateFromData(const WheelIntegrationStateData &data) {
    WheelIntegrationState state{};
    WheelPreintegrationBase::stateFromData(data, state);
    state.sodo = data.mix[9];
    return state;
}

void WheelPreintegrationEarthOdo::constructState(const double *const *parameters, WheelIntegrationState &state0,
                                                 WheelIntegrationState &state1) {
    WheelIntegrationStateData data0{}, data1{};
    memcpy(data0.pose, parameters[0], sizeof(double) * NUM_POSE);
    memcpy(data0.mix, parameters[1], sizeof(double) * NUM_MIX);
    memcpy(data1.pose, parameters[2], sizeof(double) * NUM_POSE);
    memcpy(data1.mix, parameters[3], sizeof(double) * NUM_MIX);
    state0 = stateFromData(data0);
    state1 = stateFromData(data1);
}

int WheelPreintegrationEarthOdo::imuErrorNumResiduals() { return NUM_ERROR_RESIDUAL; }
std::vector<int> WheelPreintegrationEarthOdo::imuErrorNumBlocksParameters() { return std::vector<int>{NUM_MIX}; }

void WheelPreintegrationEarthOdo::imuErrorEvaluate(const double *const *parameters, double *residuals) {
    residuals[0] = parameters[0][3] / IMU_GRY_BIAS_STD;
    residuals[1] = parameters[0][4] / IMU_GRY_BIAS_STD;
    residuals[2] = parameters[0][5] / IMU_GRY_BIAS_STD;
    residuals[3] = parameters[0][6] / IMU_ACC_BIAS_STD;
    residuals[4] = parameters[0][7] / IMU_ACC_BIAS_STD;
    residuals[5] = parameters[0][8] / IMU_ACC_BIAS_STD;
    residuals[6] = parameters[0][9] / ODO_SCALE_STD;
}

void WheelPreintegrationEarthOdo::imuErrorJacobian(double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_ERROR_RESIDUAL, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();
    jaco(0, 3) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(1, 4) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(2, 5) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(3, 6) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(4, 7) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(5, 8) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(6, 9) = 1.0 / ODO_SCALE_STD;
}

void WheelPreintegrationEarthOdo::integrationProcess(unsigned long index) {
    IMU imu_pre = compensationBias(imu_buffer_[index - 1]);
    IMU imu_cur = compensationBias(imu_buffer_[index]);

    double dt = imu_cur.dt;
    delta_time_ += dt;
    end_time_           = imu_cur.time;
    current_state_.time = imu_cur.time;

    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    Vector3d dv_cor_g = (gravity_ - 2.0 * iewn_.cross(current_state_.v)) * dt;

    Vector3d dnn    = -iewn_ * dt;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn);
    Vector3d dvel   = 0.5 * (Matrix3d::Identity() + qnn.toRotationMatrix()) * current_state_.q.toRotationMatrix() *
                    dvfb + dv_cor_g;
    current_state_.p += dt * current_state_.v + 0.5 * dt * dvel;
    current_state_.v += dvel;
    pn_.emplace_back(std::make_pair(dt, current_state_.p));

    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);
    current_state_.q = qnn * current_state_.q * Rotation::rotvec2quaternion(dtheta);
    current_state_.q.normalize();

    // preintegration delta
    dnn           = -(delta_time_ - 0.5 * dt) * iewn_;
    Matrix3d cbbe = (q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix();
    Vector3d dsodo = Vector3d(imu_cur.odovel, 0, 0);
    delta_state_.s += cbbe * (cvb_ * dsodo * (1 + delta_state_.sodo) -
                              Rotation::rotvec2quaternion(imu_cur.dtheta).toRotationMatrix() * lodo_ + lodo_);
    dvel = cbbe * dvfb;
    delta_state_.p += dt * delta_state_.v + 0.5 * dt * dvel;
    delta_state_.v += dvel;
    delta_state_.q *= Rotation::rotvec2quaternion(dtheta);
    delta_state_.q.normalize();

    updateJacobianAndCovariance(imu_pre, imu_cur);
}

void WheelPreintegrationEarthOdo::resetState(const WheelIntegrationState &state) { resetState(state, NUM_STATE); }

void WheelPreintegrationEarthOdo::updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) {
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE);
    double dt          = imu_cur.dt;
    Vector3d dnn       = -iewn_ * delta_time_;
    Matrix3d cbb0 = -(q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix();

    phi.block<3, 3>(0, 0)   = Matrix3d::Identity();
    phi.block<3, 3>(0, 3)   = Matrix3d::Identity() * dt;
    phi.block<3, 3>(3, 3)   = Matrix3d::Identity();
    phi.block<3, 3>(3, 6)   = cbb0 * Rotation::skewSymmetric(imu_cur.dvel);
    phi.block<3, 3>(3, 12)  = cbb0 * dt;
    phi.block<3, 3>(6, 6)   = Matrix3d::Identity() - Rotation::skewSymmetric(imu_cur.dtheta);
    phi.block<3, 3>(6, 9)   = -Matrix3d::Identity() * dt;
    phi.block<3, 3>(9, 9)   = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);
    phi.block<3, 3>(12, 12) = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);

    Vector3d dsodo  = Vector3d(imu_cur.odovel, 0, 0);
    Vector3d stheta = cvb_ * dsodo * (1 + delta_state_.sodo) - imu_cur.dtheta.cross(lodo_);
    phi.block<3, 3>(15, 6)  = cbb0 * Rotation::skewSymmetric(stheta);
    phi.block<3, 3>(15, 9)  = cbb0 * Rotation::skewSymmetric(lodo_) * dt;
    phi.block<3, 3>(15, 15) = Matrix3d::Identity();
    phi.block<3, 1>(15, 18) = -cbb0 * cvb_ * dsodo;
    phi(18, 18)             = 1.0;
    jacobian_               = phi * jacobian_;

    Eigen::MatrixXd gt = Eigen::MatrixXd::Zero(NUM_STATE, NUM_NOISE);
    gt.block<3, 3>(3, 3)   = cbb0;
    gt.block<3, 3>(6, 0)   = -Matrix3d::Identity();
    gt.block<3, 3>(9, 6)   = Matrix3d::Identity();
    gt.block<3, 3>(12, 9)  = Matrix3d::Identity();
    gt.block<3, 3>(15, 0)  = cbb0 * Rotation::skewSymmetric(lodo_);
    gt.block<3, 3>(15, 12) = cbb0 * cvb_ * (1 + delta_state_.sodo);
    gt(18, 15)             = 1.0;
    Eigen::MatrixXd Qk =
        0.5 * dt * (phi * gt * noise_ * gt.transpose() + gt * noise_ * gt.transpose() * phi.transpose());
    covariance_ = phi * covariance_ * phi.transpose() + Qk;
}

void WheelPreintegrationEarthOdo::resetState(const WheelIntegrationState &state, int num) {
    delta_time_ = 0;
    delta_state_.p.setZero();
    delta_state_.q.setIdentity();
    delta_state_.v.setZero();
    delta_state_.s.setZero();
    delta_state_.bg   = state.bg;
    delta_state_.ba   = state.ba;
    delta_state_.sodo = state.sodo;
    jacobian_.setIdentity(num, num);
    covariance_.setZero(num, num);
    q0_        = current_state_.q;
    iewn_      = Earth::iewn(parameters_->station, current_state_.p);
    iewn_skew_ = Rotation::skewSymmetric(iewn_);
    pn_.clear();
}

void WheelPreintegrationEarthOdo::setNoiseMatrix() {
    noise_.setIdentity(NUM_NOISE, NUM_NOISE);
    noise_.block<3, 3>(0, 0) *= parameters_->gyr_arw * parameters_->gyr_arw; // nw
    noise_.block<3, 3>(3, 3) *= parameters_->acc_vrw * parameters_->acc_vrw; // na
    noise_.block<3, 3>(6, 6) *=
        2 * parameters_->gyr_bias_std * parameters_->gyr_bias_std / parameters_->corr_time; // nbg
    noise_.block<3, 3>(9, 9) *=
        2 * parameters_->acc_bias_std * parameters_->acc_bias_std / parameters_->corr_time; // nba
    noise_(12, 12) *= parameters_->odo_std[0] * parameters_->odo_std[0];                    // nodo
    noise_(13, 13) *= parameters_->odo_std[1] * parameters_->odo_std[1];                    // nodo
    noise_(14, 14) *= parameters_->odo_std[2] * parameters_->odo_std[2];                    // nodo
    noise_(15, 15) *= parameters_->odo_srw * parameters_->odo_srw;                          // nsodo
}
