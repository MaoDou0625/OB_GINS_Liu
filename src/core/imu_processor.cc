#include "src/core/imu_processor.h"
#include <glog/logging.h>
#include "src/fileio/imufileloader.h"
#include "src/fileio/filesaver.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/factors/WheelNHCFactor.h"
#include "src/factors/WheelSpeedFactor.h"
#include "src/factors/BiasRandomWalkFactor.h"
#include "src/factors/PriorFactors.h"

namespace ob_gins {

int findControlPointIndex(double t, double t0, double dt, int max_idx) {
    return static_cast<int>(std::floor((t - dt - t0) / dt));
}

void ImuProcessor::SaveErrors(const std::string& output_path, const std::vector<spline::ControlPoint>& control_points, double spline_dt, double t_start_global) {
    if (bg_.empty() || bg_.size() != control_points.size()) return;

    std::string file_name = output_path + "/errors_" + name_ + ".txt";
    // Columns: t, bg(3), ba(3), lever_arm(3), q_body_imu(4)
    FileSaver saver(file_name, 1 + 3 + 3 + 3 + 4);

    for (size_t i = 0; i < control_points.size(); ++i) {
        double t = control_points[i].timestamp();
        
        std::vector<double> data;
        data.push_back(t);
        
        data.push_back(bg_[i].x());
        data.push_back(bg_[i].y());
        data.push_back(bg_[i].z());

        data.push_back(ba_[i].x());
        data.push_back(ba_[i].y());
        data.push_back(ba_[i].z());

        data.push_back(l_body_sensor_.x());
        data.push_back(l_body_sensor_.y());
        data.push_back(l_body_sensor_.z());

        data.push_back(q_body_imu_.x());
        data.push_back(q_body_imu_.y());
        data.push_back(q_body_imu_.z());
        data.push_back(q_body_imu_.w());

        saver.dump(data);
    }
    saver.close();
    LOG(INFO) << "Saved errors for " << name_ << " to " << file_name;
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
        
        // Load Correlation Times first (needed for Bias RW conversion)
        if (n["accel_corr_time"]) 
            acc_corr_time_ = std::max(1.0, n["accel_corr_time"].as<double>());
        if (n["gyro_corr_time"]) 
            gyr_corr_time_ = std::max(1.0, n["gyro_corr_time"].as<double>());

        double g = 9.80665;
        double d2r = M_PI / 180.0;

        // Accel Noise: ug/sqrt(Hz) -> m/s^2 (discrete sigma)
        // val (ug/rtHz) * 1e-6 * g -> m/s^2/rtHz * sqrt(rate) -> m/s^2
        if (n["accel_noise"]) {
            double val = n["accel_noise"].as<double>();
            acc_noise_ = val * 1.0e-6 * g * std::sqrt(rate_hz_);
        }

        // Gyro Noise: deg/s/sqrt(Hz) -> rad/s (discrete sigma)
        // val (deg/s/rtHz) * d2r -> rad/s/rtHz * sqrt(rate) -> rad/s
        if (n["gyro_noise"]) {
            double val = n["gyro_noise"].as<double>();
            gyr_noise_ = val * d2r * std::sqrt(rate_hz_);
        }

        // Accel Bias Instability: ug -> Driving Noise Density (m/s^3/sqrt(Hz) equivalent)
        // val (ug) * 1e-6 * g -> m/s^2 (sigma_bias)
        // sigma_driving = sigma_bias * sqrt(2/tau)
        if (n["accel_bias_rw"]) {
            double val = n["accel_bias_rw"].as<double>();
            double sigma_b = val * 1.0e-6 * g;
            acc_bias_rw_ = sigma_b * std::sqrt(2.0 / acc_corr_time_);
        }

        // Gyro Bias Instability: deg/h -> Driving Noise Density (rad/s^2/sqrt(Hz) equivalent)
        // val (deg/h) * d2r / 3600 -> rad/s (sigma_bias)
        // sigma_driving = sigma_bias * sqrt(2/tau)
        if (n["gyro_bias_rw"]) {
            double val = n["gyro_bias_rw"].as<double>();
            double sigma_b = val * d2r / 3600.0;
            gyr_bias_rw_ = sigma_b * std::sqrt(2.0 / gyr_corr_time_);
        }
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
    if (bg_.empty()) {
        bg_.resize(control_points.size(), Eigen::Vector3d::Zero());
        ba_.resize(control_points.size(), Eigen::Vector3d::Zero());
    }

    problem.AddParameterBlock(q_body_imu_.coeffs().data(), 4);
    problem.SetManifold(q_body_imu_.coeffs().data(), new ceres::EigenQuaternionManifold());
    problem.AddResidualBlock(factors::RotationPriorFactor::Create(q_body_imu_initial_, 0.01), nullptr, q_body_imu_.coeffs().data());

    problem.AddParameterBlock(l_body_sensor_.data(), 3);
    problem.AddResidualBlock(factors::LeverArmPriorFactor::Create(l_body_sensor_, 0.05), nullptr, l_body_sensor_.data());

    for (size_t i = 0; i < control_points.size(); ++i) {
        problem.AddParameterBlock(bg_[i].data(), 3);
        problem.AddParameterBlock(ba_[i].data(), 3);
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

    problem.AddParameterBlock(q_body_imu_.coeffs().data(), 4);
    problem.SetManifold(q_body_imu_.coeffs().data(), new ceres::EigenQuaternionManifold());
    problem.AddParameterBlock(l_body_sensor_.data(), 3);

    problem.AddParameterBlock(&wheel_radius_, 1);
    problem.SetParameterBlockConstant(&wheel_radius_);

    problem.AddResidualBlock(factors::RotationPriorFactor::Create(q_body_imu_initial_, 0.01), nullptr, q_body_imu_.coeffs().data());
    problem.AddResidualBlock(factors::LeverArmPriorFactor::Create(l_body_sensor_, 0.05), nullptr, l_body_sensor_.data());

    for (size_t i = 0; i < control_points.size(); ++i) {
        problem.AddParameterBlock(bg_[i].data(), 3);
        problem.AddParameterBlock(ba_[i].data(), 3);
    }

    for (const auto& imu : valid_imu_data_) {
        int k = findControlPointIndex(imu.time, t0_spline, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;

        double dt = imu.dt > 0 ? imu.dt : 1.0/rate_hz_;
        if (dt < 1e-6) continue;

        Eigen::Vector3d gyro_meas = imu.dtheta / dt;
        Eigen::Vector3d accel_meas = imu.dvel / dt;

        // Add Continuous Inertial Factor (Enables Accel/Gyro Bias Estimation)
        auto* inertial_factor = factors::ContinuousInertialFactor::Create(
            imu.time, accel_meas, gyro_meas, gravity_vec, omega_ie_local,
            spline_dt, control_points[k].timestamp(), acc_noise_, gyr_noise_
        );
        problem.AddResidualBlock(inertial_factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            bg_[k].data(), bg_[k+1].data(), 
            bg_[k+2].data(), bg_[k+3].data(),
            ba_[k].data(), ba_[k+1].data(), 
            ba_[k+2].data(), ba_[k+3].data(),
            l_body_sensor_.data()
        );

        auto* nhc_factor = factors::WheelNHCFactor::Create(
            imu.time, spline_dt, control_points[k].timestamp(), nhc_weight_, l_sensor_odopoint_
        );
        problem.AddResidualBlock(nhc_factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            q_body_imu_.coeffs().data(),
            l_body_sensor_.data()
        );

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