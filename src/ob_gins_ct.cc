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
    
    // ... (rest of main function) ...
