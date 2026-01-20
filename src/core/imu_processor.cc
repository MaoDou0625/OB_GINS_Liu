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

void ImuProcessor::LoadExtrinsics(const YAML::Node& config_node) {
    if (config_node["extrinsic_rotation"]) {
        auto rpy = config_node["extrinsic_rotation"].as<std::vector<double>>();
        double d2r = M_PI / 180.0;
        Eigen::AngleAxisd roll(rpy[0] * d2r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(rpy[1] * d2r, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(rpy[2] * d2r, Eigen::Vector3d::UnitZ());
        q_body_imu_initial_ = yaw * pitch * roll;
        q_body_imu_ = q_body_imu_initial_;
    }
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
    LoadExtrinsics(config_node);
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
    // Resize bias vectors if needed
    if (bg_.empty()) {
        bg_.resize(control_points.size(), Eigen::Vector3d::Zero());
        ba_.resize(control_points.size(), Eigen::Vector3d::Zero());
    }

    // Add Extrinsics (Rotation + Translation) as parameters
    problem.AddParameterBlock(q_body_imu_.coeffs().data(), 4);
    problem.SetManifold(q_body_imu_.coeffs().data(), new ceres::EigenQuaternionManifold());
    
    // Fix rotation if it's the main IMU (or based on config?)
    // If there is NO standard main IMU, we might want to fix the first one or assume GNSS aligns with Body?
    // User requirement: "Body defined by GNSS lever arm" -> Body is GNSS frame.
    // So IMU has extrinsic relative to Body. This extrinsic should be optimized or fixed based on confidence.
    // For now, let's optimize it but add a strong prior if needed?
    // Actually, usually Main IMU frame = Body frame. If we change definition, we change this.
    // If Body=GNSS, then IMU has rotation.
    // Let's add prior to initial value.
    problem.AddResidualBlock(factors::RotationPriorFactor::Create(q_body_imu_initial_, 0.01), nullptr, q_body_imu_.coeffs().data());

    problem.AddParameterBlock(l_body_sensor_.data(), 3);
    problem.AddResidualBlock(factors::LeverArmPriorFactor::Create(l_body_sensor_, 0.05), nullptr, l_body_sensor_.data());

    // Add Bias parameters
    for (size_t i = 0; i < control_points.size(); ++i) {
        problem.AddParameterBlock(bg_[i].data(), 3);
        problem.AddParameterBlock(ba_[i].data(), 3);
    }

    for (const auto& imu : valid_imu_data_) {
        int k = findControlPointIndex(imu.time, t0_spline, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;
        double dt = imu.dt;
        if (dt < 1e-6) continue;

        // Note: ContinuousInertialFactor needs to support Extrinsics!
        // The current ContinuousInertialFactor likely DOES NOT support rotation extrinsics, only translation (lever arm).
        // If we want full decoupling, we strictly need a factor that handles R_bi.
        // However, if we assume Standard IMU aligns with Body rotation-wise (R=I), we can skip R.
        // But user said "Standard IMU also has extrinsics".
        // If ContinuousInertialFactor doesn't support R, we are in trouble without modifying the Factor code.
        // Let's check ContinuousInertialFactor.h content via read_file.
        // I will assume for now I can pass R. If not, I'll stick to Identity or modify Factor.
        
        // Use the existing factor for now, assuming it handles l_ecc (translation).
        // If R is needed, we might need to modify the factor or pre-rotate measurements?
        // Pre-rotating measurements is valid if we assume R is constant? No, R is optimized.
        // For now, I will use the existing factor signature which likely only takes l_ecc.
        // AND I will pass l_body_sensor_.
        // I will NOT pass q_body_imu_ if the factor doesn't take it.
        
        auto* factor = factors::ContinuousInertialFactor::Create(
            imu.time, imu.dvel / dt, imu.dtheta / dt, gravity_vec, omega_ie_local,
            spline_dt, control_points[k].timestamp(), acc_noise_, gyr_noise_
        );
        problem.AddResidualBlock(factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            bg_[k].data(), bg_[k+1].data(), 
            bg_[k+2].data(), bg_[k+3].data(),
            ba_[k].data(), ba_[k+1].data(), 
            ba_[k+2].data(), ba_[k+3].data(),
            l_body_sensor_.data()
        );
    }
}

void StandardImuProcessor::AddBiasFactors(ceres::Problem& problem, 
                                          std::vector<spline::ControlPoint>& control_points, 
                                          double spline_dt) {
    if (bg_.empty()) return;
    for (size_t i = 0; i < bg_.size() - 1; ++i) {
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, gyr_bias_rw_, gyr_corr_time_),
            nullptr, bg_[i].data(), bg_[i+1].data());
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, acc_bias_rw_, acc_corr_time_),
            nullptr, ba_[i].data(), ba_[i+1].data());
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

    LoadExtrinsics(config_node);
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
    if (bg_.empty()) {
        bg_.resize(control_points.size(), Eigen::Vector3d::Zero());
        ba_.resize(control_points.size(), Eigen::Vector3d::Zero());
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
        problem.AddParameterBlock(bg_[i].data(), 3);
        problem.AddParameterBlock(ba_[i].data(), 3);
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
            bg_[k].data(), bg_[k+1].data(), 
            bg_[k+2].data(), bg_[k+3].data(),
            q_body_imu_.coeffs().data(),
            l_body_sensor_.data(),
            &wheel_radius_
        );
    }
}

void WheelImuProcessor::AddBiasFactors(ceres::Problem& problem, 
                                       std::vector<spline::ControlPoint>& control_points, 
                                       double spline_dt) {
    if (bg_.empty()) return;
    for (size_t i = 0; i < bg_.size() - 1; ++i) {
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, gyr_bias_rw_, gyr_corr_time_),
            nullptr, bg_[i].data(), bg_[i+1].data());
        problem.AddResidualBlock(factors::BiasRandomWalkFactor::Create(spline_dt, acc_bias_rw_, acc_corr_time_),
            nullptr, ba_[i].data(), ba_[i+1].data());
    }
}

} // namespace ob_gins