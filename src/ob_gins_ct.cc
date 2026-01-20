#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <filesystem>
#include <algorithm>

#include <glog/logging.h>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <sophus/se3.hpp>

#include "src/common/types.h"
#include "src/common/earth.h"
#include "src/fileio/imufileloader.h"
#include "src/fileio/gnssfileloader.h"
#include "src/fileio/filesaver.h"
#include "src/spline/SplineInitializer.h"
#include "src/spline/BSplineEvaluator.h"
#include "src/spline/SophusSE3Manifold.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/factors/ContinuousGnssFactor.h"
#include "src/factors/BiasRandomWalkFactor.h"
#include "src/factors/PriorFactors.h"
#include "src/core/imu_processor.h"

using namespace ob_gins;
using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    if (argc != 2) {
        LOG(ERROR) << "Usage: ./ob_gins_ct <config_file_path>";
        return -1;
    }

    LOG(INFO) << "OB_GINS Continuous Time Optimization";
    
    // 1. Load Config
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception& e) {
        LOG(ERROR) << "Failed to load config: " << e.what();
        return -1;
    }

    // 设置调试级别
    if (config["debug"] && config["debug"]["level"]) {
        FLAGS_v = config["debug"]["level"].as<int>();
    }

    std::string output_path = config["outputpath"].as<std::string>();
    if (!std::filesystem::exists(output_path)) {
        std::filesystem::create_directories(output_path);
    }

    // 2. Load Data - GNSS First (for initial time window and origin)
    LOG(INFO) << "Loading GNSS data...";
    std::string gnss_path = config["gnssfile"].as<std::string>();
    GnssFileLoader gnss_loader(gnss_path);

    std::vector<GNSS> gnss_data;
    while (!gnss_loader.isEof()) {
        gnss_data.push_back(gnss_loader.next());
    }
    if (gnss_data.empty()) {
        LOG(ERROR) << "Empty GNSS data loaded.";
        return -1;
    }

    double t_start_global = gnss_data.front().time;
    double t_end_global = gnss_data.back().time;

    if (config["starttime"]) t_start_global = std::max(t_start_global, config["starttime"].as<double>());
    if (config["endtime"]) t_end_global = std::min(t_end_global, config["endtime"].as<double>());

    LOG(INFO) << "Initial Time window from GNSS: " << std::fixed << t_start_global << " to " << t_end_global 
              << " (Duration: " << (t_end_global - t_start_global) << "s)";

    // Store all ImuProcessors
    std::vector<std::unique_ptr<ImuProcessor>> imu_processors;
    
    // Default bias random walk noise values
    double global_acc_bias_rw = 1.0e-4;
    double global_gyr_bias_rw = 1.0e-5;

    // Iterate through config to find all IMU entries
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string key = it->first.as<std::string>();
        if (key.rfind("imu", 0) == 0 && it->second.IsMap()) { // Check if key starts with "imu" and is a map
            if (it->second["type"]) {
                std::string type = it->second["type"].as<std::string>();
                if (type == "standard") {
                    auto processor = std::make_unique<StandardImuProcessor>();
                    if (processor->LoadConfig(it->second, key)) {
                        imu_processors.push_back(std::move(processor));
                        // If this is the main IMU, use its bias random walk for global bias terms
                        if (key == "imu_main" && it->second["imunoise"]) {
                            const auto& noise_node = it->second["imunoise"];
                            if (noise_node["accel_bias_rw"]) global_acc_bias_rw = noise_node["accel_bias_rw"].as<double>();
                            else if (noise_node["abstd"]) global_acc_bias_rw = noise_node["abstd"].as<double>();
                            
                            if (noise_node["gyro_bias_rw"]) global_gyr_bias_rw = noise_node["gyro_bias_rw"].as<double>();
                            else if (noise_node["gbstd"]) global_gyr_bias_rw = noise_node["gbstd"].as<double>();
                        }
                    } else {
                        LOG(ERROR) << "Failed to load config for Standard IMU: " << key;
                    }
                } else if (type == "wheel") {
                    auto processor = std::make_unique<WheelImuProcessor>();
                    if (processor->LoadConfig(it->second, key)) {
                        imu_processors.push_back(std::move(processor));
                    } else {
                        LOG(ERROR) << "Failed to load config for Wheel IMU: " << key;
                    }
                } else {
                    LOG(WARNING) << "Unknown IMU type '" << type << "' for sensor: " << key;
                }
            } else {
                LOG(WARNING) << "IMU config for '" << key << "' is missing 'type' field. Skipping.";
            }
        }
    }

    if (imu_processors.empty()) {
        LOG(ERROR) << "No IMU configurations found or loaded.";
        return -1;
    }

    // Load data for all IMUs and adjust global time window
    for (const auto& processor : imu_processors) {
        if (!processor->LoadData(t_start_global, t_end_global)) {
            LOG(ERROR) << "Failed to load data for IMU: " << processor->GetName();
            return -1;
        }
        // Adjust global time window based on actual loaded IMU data
        if (!processor->GetImuData().empty()) {
            t_start_global = std::max(t_start_global, processor->GetImuData().front().time);
            t_end_global = std::min(t_end_global, processor->GetImuData().back().time);
        }
    }
    
    // Filter GNSS data based on final global time window
    GNSS origin_gnss;
    bool origin_set = false;
    std::vector<GNSS> valid_gnss;
    
    for (const auto& gnss : gnss_data) {
        if (gnss.time >= t_start_global) {
            if (!origin_set) {
                origin_gnss = gnss;
                origin_set = true;
            }
            if (gnss.time <= t_end_global) {
                valid_gnss.push_back(gnss);
            }
        }
    }
    
    if (!origin_set || valid_gnss.empty()) {
        LOG(ERROR) << "No valid GNSS data in final time window [" << t_start_global << ", " << t_end_global << "].";
        return -1;
    }

    LOG(INFO) << "Final Time window: " << std::fixed << t_start_global << " to " << t_end_global 
              << " (Duration: " << (t_end_global - t_start_global) << "s)";
    
    // 3. Initialize Spline and Earth Model
    double spline_dt = 1.0; 
    if (config["kf_interval_sec"]) spline_dt = config["kf_interval_sec"].as<double>();
    
    Earth earth;
    Vector3d gravity_l, omega_ie_l;
    // 以第一点 GNSS 作为局部坐标系原点
    Vector3d origin_ecef = earth.blh2ecef(valid_gnss.front().blh);

    if (config["isearth"] && config["isearth"].as<bool>()) {
        double g = earth.gravity(valid_gnss.front().blh);
        gravity_l << 0, 0, -g; // 导航系(ENU)下的重力
        omega_ie_l = earth.iewn(valid_gnss.front().blh(0)); // 导航系下的地球自转
    } else {
        gravity_l << 0, 0, -9.80665;
        omega_ie_l.setZero();
    }

    // 4. Build Optimization Problem
    ceres::Problem problem;
    
    // 初始化样条曲线控制点 (将 GNSS 转换为局部 ENU 进行初始化)
    std::vector<GNSS> gnss_enu = valid_gnss;
    std::vector<std::pair<double, Sophus::SE3d>> path_for_init;

    for (auto& g : gnss_enu) {
        // Use global2local to convert BLH to local frame (ENU/NED)
        // Store result in g.blh temporarily
        g.blh = earth.global2local(valid_gnss.front().blh, g.blh); 
        
        // Prepare path for SplineInitializer
        // Assume identity rotation for initialization if not available
        path_for_init.emplace_back(g.time, Sophus::SE3d(Eigen::Quaterniond::Identity(), g.blh));
    }

    std::vector<ControlPoint> control_points = SplineInitializer::InitializeFromPath(path_for_init, spline_dt);

    // 设置位姿流形
    for (auto& cp : control_points) {
        problem.AddParameterBlock(cp.pose_data(), 7);
        problem.SetManifold(cp.pose_data(), new SophusSE3Manifold());
        // Biases are now managed by ImuProcessors individually
    }

    // 添加 GNSS 因子
    // Body Frame is defined as GNSS Center, so Lever Arm is ZERO.
    Eigen::Vector3d gnss_lever_arm = Eigen::Vector3d::Zero();
    problem.AddParameterBlock(gnss_lever_arm.data(), 3);
    problem.SetParameterBlockConstant(gnss_lever_arm.data());

    Eigen::Vector3d gnss_std(1.0/0.1, 1.0/0.1, 1.0/0.2);
    Matrix3d gnss_sqrt_info = gnss_std.asDiagonal(); 
    for (const auto& gnss : gnss_enu) {
        int k = findControlPointIndex(gnss.time, t_start_global, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;

        auto* factor = ContinuousGnssFactor::Create(gnss.time, spline_dt, t_start_global, gnss.blh, gnss_sqrt_info);
        problem.AddResidualBlock(factor, nullptr, 
            control_points[k].pose_data(), control_points[k+1].pose_data(), 
            control_points[k+2].pose_data(), control_points[k+3].pose_data(),
            gnss_lever_arm.data() // GNSS lever arm is zero
        );
    }

    // 添加所有 IMU 约束 (Standard + Wheel)
    for (auto& processor : imu_processors) {
        processor->AddFactors(problem, control_points, spline_dt, t_start_global, gravity_l, omega_ie_l);
        processor->AddBiasFactors(problem, control_points, spline_dt);
    }

    // 5. Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = config["num_iterations"] ? config["num_iterations"].as<int>() : 20;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << summary.BriefReport();

    // 6. Save Results
    std::string result_file = output_path + "/ct_trajectory.txt";
    // 8 columns: time, x, y, z, qx, qy, qz, qw
    FileSaver saver(result_file, 8); 
    
    double output_interval = config["kf_interval_sec"] ? config["kf_interval_sec"].as<double>() : 0.1;
    for (double t = t_start_global + spline_dt; t < t_end_global - spline_dt; t += output_interval) {
        int k = findControlPointIndex(t, t_start_global, spline_dt, (int)control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) continue;

        double u = (t - (t_start_global + k * spline_dt)) / spline_dt;
        auto res = BSplineEvaluator::Evaluate<double>(u, spline_dt, 
            control_points[k].pose(), control_points[k+1].pose(), 
            control_points[k+2].pose(), control_points[k+3].pose());
        
        // 保存格式: time, x, y, z, qx, qy, qz, qw
        Eigen::Vector3d p = res.pose.translation();
        Eigen::Quaterniond q = res.pose.so3().unit_quaternion();
        saver.dump({t, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()});
    }

    saver.close(); // Ensure file is flushed before python script reads it

    LOG(INFO) << "Trajectory saved to: " << result_file;

    // 7. Comparison Script
    if (config["comparison"] && config["comparison"]["enable"].as<bool>()) {
        std::string python_exe = "python"; 
        std::string script = config["comparison"]["python_script"].as<std::string>();
        std::string truth = config["comparison"]["truth_file"].as<std::string>();
        
        std::string cmd = python_exe + " " + script + " --result " + result_file + " --truth " + truth;
        LOG(INFO) << "Running comparison: " << cmd;
        int ret = std::system(cmd.c_str());
        if (ret != 0) LOG(WARNING) << "Comparison script returned non-zero code: " << ret;
    }

    return 0;
}
