#include "src/core/imu_processor.h"
#include <glog/logging.h>
#include "src/fileio/imufileloader.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/factors/WheelNHCFactor.h"
#include "src/factors/WheelSpeedFactor.h"
#include "src/factors/BiasRandomWalkFactor.h"
#include "src/factors/PriorFactors.h"

namespace ob_gins {

int findControlPointIndex(double t, double t0, double dt, int max_idx) {
    return static_cast<int>(std::floor((t - dt - t0) / dt));
}

bool ImuProcessor::LoadImuFileAndFilter(double t_start, double t_end) {
    ImuFileLoader imu_loader(file_path_, columns_, rate_hz_);
    while (!imu_loader.isEof()) {
        all_imu_data_.push_back(imu_loader.next());
    }
    if (all_imu_data_.empty()) return false;
    for (const auto& imu : all_imu_data_) {
        if (imu.time >= t_start && imu.time <= t_end) valid_imu_data_.push_back(imu);
    }
    return !valid_imu_data_.empty();
}

Eigen::Vector3d ImuProcessor::LoadLeverArm(const YAML::Node& config_node, const std::string& key) {
    if (config_node[key]) {
        auto vec = config_node[key].as<std::vector<double>>();
        if (vec.size() == 3) return Eigen::Vector3d(vec[0], vec[1], vec[2]);
    }
    return Eigen::Vector3d::Zero();
}

void ImuProcessor::LoadImuNoise(const YAML::Node& config_node) {
    if (config_node["imunoise"]) {
        const auto& n = config_node["imunoise"];
        if (n["accel_noise"]) acc_noise_ = n["accel_noise"].as<double>();
        if (n["gyro_noise"]) gyr_noise_ = n["gyro_noise"].as<double>();
        if (n["accel_bias_rw"]) acc_bias_rw_ = n["accel_bias_rw"].as<double>();
        if (n["gyro_bias_rw"]) gyr_bias_rw_ = n["gyro_bias_rw"].as<double>();
        if (n["accel_corr_time"]) 
            acc_corr_time_ = std::max(1.0, n["accel_corr_time"].as<double>());
        if (n["gyro_corr_time"]) 
            gyr_corr_time_ = std::max(1.0, n["gyro_corr_time"].as<double>());
    }
}

bool StandardImuProcessor::LoadConfig(const YAML::Node& config_node, const std::string& imu_name) {
    name_ = imu_name;
    file_path_ = config_node["file"].as<std::string>();
    columns_ = config_node["columns"].as<int>();
    rate_hz_ = config_node["rate_hz"].as<double>();
    l_body_sensor_ = LoadLeverArm(config_node, "antlever");
    LoadImuNoise(config_node);
    return true;
}

bool StandardImuProcessor::LoadData(double t_start, double t_end) {
    return LoadImuFileAndFilter(t_start, t_end);
}

void StandardImuProcessor::AddFactors(ceres::Problem& problem, 
                                      std::vector<spline::ControlPoint>& control_points, 
                                      double spline_dt, double t0_spline,
                                      const Eigen::Vector3d& gravity_vec, 
                                      const Eigen::Vector3d& omega_ie_local) {
    problem.AddParameterBlock(l_body_sensor_.data(), 3);
    if (name_ == "imu_main" || l_body_sensor_.isZero()) {
        problem.SetParameterBlockConstant(l_body_sensor_.data());
    } else {
        // 步骤 4: 为非主 IMU 的杆臂添加先验约束 (标准差设为 5cm)
        problem.AddResidualBlock(factors::LeverArmPriorFactor::Create(l_body_sensor_, 0.05), nullptr, l_body_sensor_.data());
    }

    for (const auto& imu : valid_imu_data_) {
        int k = findControlPointIndex(imu.time, t0_spline, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;
        double dt = imu.dt;
        if (dt < 1e-6) continue;

        auto* factor = factors::ContinuousInertialFactor::Create(
            imu.time, imu.dvel / dt, imu.dtheta / dt, gravity_vec, omega_ie_local,
            spline_dt, control_points[k].timestamp(), acc_noise_, gyr_noise_
        );
        problem.AddResidualBlock(factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            control_points[k].bg_data(), control_points[k+1].bg_data(), 
            control_points[k+2].bg_data(), control_points[k+3].bg_data(),
            control_points[k].ba_data(), control_points[k+1].ba_data(), 
            control_points[k+2].ba_data(), control_points[k+3].ba_data(),
            l_body_sensor_.data()
        );
    }
}

void StandardImuProcessor::AddBiasFactors(ceres::Problem& problem, 
                                          std::vector<spline::ControlPoint>& control_points, 
                                          double spline_dt) {
    for (size_t i = 0; i < control_points.size() - 1; ++i) {
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, gyr_bias_rw_, gyr_corr_time_),
            nullptr, control_points[i].bg_data(), control_points[i+1].bg_data());
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, acc_bias_rw_, acc_corr_time_),
            nullptr, control_points[i].ba_data(), control_points[i+1].ba_data());
    }
}

bool WheelImuProcessor::LoadConfig(const YAML::Node& config_node, const std::string& imu_name) {
    name_ = imu_name;
    file_path_ = config_node["file"].as<std::string>();
    columns_ = config_node["columns"].as<int>();
    rate_hz_ = config_node["rate_hz"].as<double>();
    l_body_sensor_ = LoadLeverArm(config_node, "antlever");
    l_sensor_odopoint_ = LoadLeverArm(config_node, "odolever");
    side_ = config_node["side"].as<std::string>();

    if (config_node["extrinsic_rotation"]) {
        auto rpy = config_node["extrinsic_rotation"].as<std::vector<double>>();
        double d2r = M_PI / 180.0;
        Eigen::AngleAxisd roll(rpy[0] * d2r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(rpy[1] * d2r, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(rpy[2] * d2r, Eigen::Vector3d::UnitZ());
        q_body_imu_initial_ = yaw * pitch * roll;
        q_body_imu_ = q_body_imu_initial_;
    }
    if (config_node["nhc_weight"]) nhc_weight_ = config_node["nhc_weight"].as<double>();
    if (config_node["speed_weight"]) speed_weight_ = config_node["speed_weight"].as<double>();
    if (config_node["wheel_radius"]) wheel_radius_ = config_node["wheel_radius"].as<double>();
    LoadImuNoise(config_node);
    return true;
}

bool WheelImuProcessor::LoadData(double t_start, double t_end) {
    if (!LoadImuFileAndFilter(t_start, t_end)) return false;
    for (auto& imu : valid_imu_data_) {
        if (side_ == "right") imu.dtheta.z() *= -1.0;
    }
    return true;
}

void WheelImuProcessor::AddFactors(ceres::Problem& problem, 
                                   std::vector<spline::ControlPoint>& control_points, 
                                   double spline_dt, double t0_spline,
                                   const Eigen::Vector3d& gravity_vec, 
                                   const Eigen::Vector3d& omega_ie_local) {
    if (wheel_bg_.empty()) {
        wheel_bg_.resize(control_points.size(), Eigen::Vector3d::Zero());
        wheel_ba_.resize(control_points.size(), Eigen::Vector3d::Zero());
    }

    // 1. 将安装旋转和杆臂添加为参数块
    problem.AddParameterBlock(q_body_imu_.coeffs().data(), 4);
    // 使用 EigenQuaternionManifold 以匹配 Eigen 的 [x, y, z, w] 存储顺序
    problem.SetManifold(q_body_imu_.coeffs().data(), new ceres::EigenQuaternionManifold());
    problem.AddParameterBlock(l_body_sensor_.data(), 3);

    // 轮径参数块
    problem.AddParameterBlock(&wheel_radius_, 1);
    problem.SetParameterBlockConstant(&wheel_radius_); // 默认固定，后期可开启优化

    // 2. 添加先验约束，防止参数在观测不足时发散
    problem.AddResidualBlock(factors::RotationPriorFactor::Create(q_body_imu_initial_, 0.01), nullptr, q_body_imu_.coeffs().data());
    problem.AddResidualBlock(factors::LeverArmPriorFactor::Create(l_body_sensor_, 0.05), nullptr, l_body_sensor_.data());

    for (size_t i = 0; i < control_points.size(); ++i) {
        problem.AddParameterBlock(wheel_bg_[i].data(), 3);
        problem.AddParameterBlock(wheel_ba_[i].data(), 3);
    }

    for (const auto& imu : valid_imu_data_) {
        int k = findControlPointIndex(imu.time, t0_spline, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;

        // 添加 NHC 因子 (侧向和垂向约束)
        auto* nhc_factor = factors::WheelNHCFactor::Create(
            imu.time, spline_dt, control_points[k].timestamp(), nhc_weight_, l_sensor_odopoint_
        );
        problem.AddResidualBlock(nhc_factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            q_body_imu_.coeffs().data(),
            l_body_sensor_.data()
        );

        // 添加里程计速度因子 (前向约束)，传入完整角速度向量以支持安装角优化
        double dt = imu.dt > 0 ? imu.dt : 1.0/rate_hz_;
        Eigen::Vector3d gyro_meas = imu.dtheta / dt;

        auto* speed_factor = factors::WheelSpeedFactor::Create(
            imu.time, spline_dt, control_points[k].timestamp(), gyro_meas, speed_weight_, l_sensor_odopoint_
        );
        problem.AddResidualBlock(speed_factor, nullptr,
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            wheel_bg_[k].data(), wheel_bg_[k+1].data(), 
            wheel_bg_[k+2].data(), wheel_bg_[k+3].data(),
            q_body_imu_.coeffs().data(),
            l_body_sensor_.data(),
            &wheel_radius_
        );
    }
}

void WheelImuProcessor::AddBiasFactors(ceres::Problem& problem, 
                                       std::vector<spline::ControlPoint>& control_points, 
                                       double spline_dt) {
    if (wheel_bg_.empty()) return;
    for (size_t i = 0; i < wheel_bg_.size() - 1; ++i) {
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, gyr_bias_rw_, gyr_corr_time_),
            nullptr, wheel_bg_[i].data(), wheel_bg_[i+1].data());
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, acc_bias_rw_, acc_corr_time_),
            nullptr, wheel_ba_[i].data(), wheel_ba_[i+1].data());
    }
}

} // namespace ob_gins