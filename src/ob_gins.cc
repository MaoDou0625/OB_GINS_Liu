/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/common/earth.h"
#include "src/common/types.h"
#include "src/common/logging.h"
#include "src/common/debug.h"
#include "src/common/angle.h"
#include "src/core/imu_chain.h"
#include "src/fileio/gnssfileloader.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_manifold.h"
#include "src/core/preintegration_factory.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

// #define INTEGRATION_LENGTH 1.0
// MINIMUM_INTERVAL is now defined in types.h
#define INTEGRATION_LENGTH 1.0

// YAML helper: tolerant boolean parsing
static bool yamlToBool(const YAML::Node &node, bool def = false) {
    try {
        if (!node || node.IsNull()) return def;
        if (node.IsScalar()) {
            try {
                std::string t = node.as<std::string>();
                std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return (char) std::tolower(c); });
                if (t == "true" || t == "1" || t == "yes" || t == "on") return true;
                if (t == "false" || t == "0" || t == "no" || t == "off") return false;
            } catch (...) {
            }
            try { int vi = node.as<int>(); return vi != 0; } catch (...) {}
            try { return node.as<bool>(); } catch (...) { return def; }
        } else {
            try { return node.as<bool>(); } catch (...) { return def; }
        }
    } catch (...) { return def; }
    return def;
}

int main(int argc, char *argv[]) {
    initializePreintegrationFactory();
    
    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";
    Logging::initialization(argv, /*logtostderr=*/true, /*logtofile=*/false);

    auto ts = std::chrono::steady_clock::now();

    // Load configuration
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // Global parameters
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    double integration_length = INTEGRATION_LENGTH;
    if (config["kf_interval_sec"]) {
        try { integration_length = config["kf_interval_sec"].as<double>(); } catch (...) {}
    }

    int num_iterations     = config["num_iterations"].as<int>();
    bool is_outlier_culling = yamlToBool(config["is_outlier_culling"], false);

    // Debug controls
    bool dbg_enable = false;
    int  dbg_level  = 0;
    if (config["debug"]) {
        auto dbg = config["debug"];
        dbg_enable = yamlToBool(dbg["enable"], false);
        if (dbg["level"])  dbg_level  = dbg["level"].as<int>();
    }
    Debug::set(dbg_enable, dbg_level);

    // Run mode
    // Dynamic IMU chain loading replaces explicit run_mode checks
    /*
    bool use_main = false, use_wheel_left = false, use_wheel_right = false;
    if (config["run_mode"]) {
        auto rm = config["run_mode"];
        use_main        = yamlToBool(rm["imu_main_enable"], use_main);
        use_wheel_left  = yamlToBool(rm["wheel_left_enable"], use_wheel_left);
        use_wheel_right = yamlToBool(rm["wheel_right_enable"], use_wheel_right);
    }
    */

    // IO paths
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();

    // Ensure output directory exists
    try {
        if (!outputpath.empty()) {
            std::filesystem::path outdir(outputpath);
            if (!std::filesystem::exists(outdir)) {
                std::filesystem::create_directories(outdir);
                std::cout << "[info] output path not found. Created: " << outdir.string() << std::endl;
            }
        }
    } catch (const std::exception &e) {
        std::cout << "[error] failed to create output path: " << outputpath << ", reason: " << e.what() << std::endl;
    }

    // GNSS loader
    GnssFileLoader gnssfile(gnsspath);

    // Construct IMU chains
    std::vector<std::unique_ptr<ImuChain>> chains;
    
    // Dynamically load IMU chains from config
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string name = it->first.as<std::string>();
        YAML::Node node = it->second;

        // Check if node is a map and has a "type" field
        if (node.Type() == YAML::NodeType::Map && node["type"]) {
            std::string type = node["type"].as<std::string>();
            
            // Normalize type string if needed, currently strictly checking
            if (type == "standard" || type == "wheel") {
                auto chain = std::make_unique<ImuChain>(name, node, config);
                if (chain->isEnabled()) {
                    chains.emplace_back(std::move(chain));
                    std::cout << "[info] Added IMU chain: " << name << " (type: " << type << ")" << std::endl;
                }
            }
        }
    }

    /*
    auto add_chain = [&](const std::string &name, const YAML::Node &node) {
        if (!node) return;
        auto chain = std::make_unique<ImuChain>(name, node, config);
        if (chain->isEnabled()) chains.emplace_back(std::move(chain));
    };
    if (use_main && config["imu_main"])       add_chain("main",       config["imu_main"]);
    if (use_wheel_left && config["imu_wheel_left"])  add_chain("wheel_left",  config["imu_wheel_left"]);
    if (use_wheel_right && config["imu_wheel_right"]) add_chain("wheel_right", config["imu_wheel_right"]);
    */

    if (chains.empty()) {
        std::cout << "No IMU chain enabled" << std::endl;
        return -1;
    }

    // --- Initial Time and GNSS Setup ---

    // Determine alignment and navigation start times
    double alignment_starttime = config["starttime"].as<double>();
    double alignment_time = 0.0;
    if (config["aligntime"]) {
        alignment_time = config["aligntime"].as<double>();
    }
    double navigation_starttime = alignment_starttime;
    if (alignment_time > 0.0) {
        navigation_starttime += alignment_time;
    }

    // Find GNSS measurements for alignment and navigation start times in a single pass.
    GNSS gnss = gnssfile.next();
    GNSS align_gnss = gnss;
    GNSS nav_gnss = gnss;
    GNSS next_gnss{};
    bool has_next_after_start = false;

    // Find the last GNSS measurement at or before the alignment start time
    while (gnss.time <= alignment_starttime && !gnssfile.isEof()) {
        align_gnss = gnss;
        gnss = gnssfile.next();
    }
    
    // Perform alignment for each chain, which advances their IMU streams to navigation_starttime
    for (auto &chain : chains) chain->alignAndSync(alignment_starttime, align_gnss.blh);

    // Now, find the last GNSS measurement at or before the navigation start time,
    // continuing from where the previous search left off.
    nav_gnss = align_gnss;
    while (gnss.time <= navigation_starttime && !gnssfile.isEof()) {
        nav_gnss = gnss;
        gnss = gnssfile.next();
    }
    
    // The next GNSS measurement after the navigation start time is now in `gnss`.
    if (gnss.time > navigation_starttime) {
        next_gnss = gnss;
        has_next_after_start = true;
    }

    Vector3d station_origin = nav_gnss.blh;

    if (Debug::on(1)) {
        Vector3d blh_deg = nav_gnss.blh * R2D;
        Debug::print(1, "INIT_GNSS",
                     "t=" + std::to_string(nav_gnss.time) +
                     " lat(deg)=" + std::to_string(blh_deg[0]) +
                     " lon(deg)=" + std::to_string(blh_deg[1]) +
                     " h(m)=" + std::to_string(nav_gnss.blh[2]));
    }

    // Initialize chains at the navigation start time
    for (auto &chain : chains) chain->initializeFirstState(nav_gnss, station_origin);
    GNSS nav_gnss_local = nav_gnss;
    nav_gnss_local.blh = Earth::global2local(station_origin, nav_gnss.blh);

    // Save initial navigation output at start time
    for (auto &chain : chains) chain->writeResult(nav_gnss.time, station_origin, 0);

    std::deque<double> timelist;
    std::deque<GNSS> gnsslist;
    double sow = nav_gnss.time;
    if (Debug::on(2)) {
        Debug::print(2, "INIT_TIME",
                     "nav_gnss.t=" + std::to_string(nav_gnss.time) +
                     " sow=" + std::to_string(sow));
    }
    timelist.push_back(sow);
    gnsslist.push_back(nav_gnss_local);
    sow += integration_length;

    // GNSS outage params
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();
    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // Prime next GNSS
    if (has_next_after_start) {
        gnss = next_gnss;
    } else {
        gnss.time = std::numeric_limits<double>::infinity();  // no GNSS after start time
    }

    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;
    int lastpercent = -1;

    while (true) {
        // Exit if all chains are done
        bool active = false;
        for (auto &chain : chains) {
            auto loader = chain->getLoader();
            if (chain->isEnabled() && loader && !loader->isEof() && chain->currentImu().time <= endtime) {
                active = true;
                break;
            }
        }
        if (!active) break;

        // Feed IMUs up to the next keyframe boundary
        for (auto &chain : chains) {
            auto loader = chain->getLoader();
            if (!chain->isEnabled() || !loader) continue;
            if (!loader->isEof() && chain->currentImu().time <= sow) {
                chain->processImuUpTo(sow);
            }
        }

        // Wait until all chains have reached the boundary
        bool boundary_ready = true;
        for (auto &chain : chains) {
            auto loader = chain->getLoader();
            if (chain->isEnabled() && loader && !loader->isEof() && chain->currentImu().time <= sow) {
                boundary_ready = false;
                break;
            }
        }
        if (!boundary_ready) continue;

        // GNSS at boundary
        if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
            if (Debug::on(2)) {
                Debug::print(2, "GNSS_PUSH",
                             "sow=" + std::to_string(sow) +
                             " gnss.t=" + std::to_string(gnss.time) +
                             " diff=" + std::to_string(fabs(gnss.time - sow)));
            }
            gnsslist.push_back(gnss);
            gnss = gnssfile.next();
            while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) || (gnss.std[2] > gnssthreshold)) {
                gnss = gnssfile.next();
            }
            if (isuseoutage) {
                if (lround(gnss.time) == outagetime) {
                    std::cout << "GNSS outage at " << outagetime << " s" << std::endl;
                    for (int k = 0; k < outagelen; k++) { gnss = gnssfile.next(); }
                    outagetime += outageperiod;
                }
            }
            double g_val = Earth::gravity(gnss.blh);
            for (auto &chain : chains) chain->updateGravity(g_val);
            gnss.blh = Earth::global2local(station_origin, gnss.blh);
            if (gnssfile.isEof()) { gnss.time = 0; }
        }

        timelist.push_back(sow);

        // Build and solve optimization
        ceres::Problem::Options problem_options;
        problem_options.enable_fast_removal = true;
        ceres::Problem problem(problem_options);
        ceres::Solver solver;
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads                = 4;

        size_t max_idx = chains.front()->segmentCount();

        for (auto &chain : chains) {
            size_t local_idx = chain->segmentCount();
            chain->addParameterBlocksTo(problem, local_idx);
        }

        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
        std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id;
        for (const auto &g : gnsslist) {
            ceres::ResidualBlockId id{};
            chains.front()->addGnssFactorTo(problem, g, loss_function, timelist, &id);
            if (id) gnss_residualblock_id.emplace_back(g.time, id);
            for (size_t c = 1; c < chains.size(); ++c) {
                chains[c]->addGnssFactorTo(problem, g, loss_function, timelist, nullptr);
            }
        }

        for (auto &chain : chains) {
            size_t local_idx = chain->segmentCount();
            chain->addImuFactorsTo(problem, local_idx);
            if (local_idx > 0) chain->addBiasFactorTo(problem, local_idx);
        }

        if (last_marginalization_info && last_marginalization_info->isValid()) {
            auto factor = new MarginalizationFactor(last_marginalization_info);
            problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
        }

        options.max_num_iterations = num_iterations / 4;
        solver.Solve(options, &problem, &summary);

        // GNSS outlier culling on the primary chain
        if (is_outlier_culling && !gnss_residualblock_id.empty()) {
            double chi2_threshold = 7.815;
            size_t K = std::min(gnsslist.size(), gnss_residualblock_id.size());
            for (size_t k = 0; k < K; k++) {
                auto id = gnss_residualblock_id[k].second;
                double cost;
                problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
                double chi2 = cost * 2;
                if (chi2 > chi2_threshold) {
                    double scale = sqrt(chi2 / chi2_threshold);
                    gnsslist[k].std *= scale;
                }
            }
            for (const auto &block : gnss_residualblock_id) problem.RemoveResidualBlock(block.second);

            // Re-add GNSS without loss
            for (const auto &g2 : gnsslist) {
                chains.front()->addGnssFactorTo(problem, g2, nullptr, timelist, nullptr);
                for (size_t c = 1; c < chains.size(); ++c) chains[c]->addGnssFactorTo(problem, g2, nullptr, timelist, nullptr);
            }
        }

        options.max_num_iterations = num_iterations * 3 / 4;
        solver.Solve(options, &problem, &summary);

        // Sync optimized states back to each chain
        for (auto &chain : chains) chain->syncStatesFromOptimizer();

        // progress output
        int percent = ((int) sow - starttime) * 100 / (endtime - starttime);
        if (percent < 0) percent = 0; if (percent > 100) percent = 100;
        if (percent != lastpercent) {
            lastpercent = percent;
            std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
            flush(std::cout);
        }

        // Marginalization when window full
        if (max_idx == static_cast<size_t>(windows)) {
            std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();

            if (last_marginalization_info && last_marginalization_info->isValid()) {
                std::vector<int> marginilized_index;
                for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                    for (auto &chain : chains) {
                        if (last_marginalization_parameter_blocks[k] == chain->getStateDataList()[0].pose ||
                            last_marginalization_parameter_blocks[k] == chain->getStateDataList()[0].mix) {
                            marginilized_index.push_back(static_cast<int>(k));
                        }
                    }
                }
                auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                marginalization_info->addResidualBlockInfo(residual);
            }

            GNSS first_gnss = gnsslist.empty() ? GNSS{} : gnsslist[0];
            for (auto &chain : chains) {
                if (chain->segmentCount() > 0) {
                    chain->addFactorsToMarginalizationInfo(marginalization_info, first_gnss);
                }
            }

            marginalization_info->marginalization();

            std::unordered_map<std::uintptr_t, double *> address;
            for (auto &chain : chains) {
                auto &states = chain->getStateDataList();
                size_t local_idx = chain->segmentCount();
                for (size_t k = 1; k <= local_idx; k++) {
                    address[reinterpret_cast<std::uintptr_t>(states[k].pose)] = states[k - 1].pose;
                    address[reinterpret_cast<std::uintptr_t>(states[k].mix)]  = states[k - 1].mix;
                }
            }
            last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
            last_marginalization_info             = std::move(marginalization_info);

            if (!gnsslist.empty() && fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) gnsslist.pop_front();
            timelist.pop_front();
            for (auto &chain : chains) chain->slideWindow(timelist, gnsslist);
        }

        // Write results
        double pub_time = timelist.back();
        for (auto &chain : chains) chain->writeResult(pub_time, station_origin);

        // Start next preintegration
        for (auto &chain : chains) chain->startNewPreintegration();

        // sow += integration_length;
        // Adaptive time stepping: find the next GNSS measurement closest to (sow + integration_length)
        if (!gnssfile.isEof()) {
            double next_target = sow + integration_length;
            
            // Advance GNSS stream until we are close to the target time
            // Stop if we pass the target time or if we are within a small tolerance window
            while (gnss.time < next_target - 0.5 * integration_length && !gnssfile.isEof()) {
                gnss = gnssfile.next();
            }

            // Now gnss is a candidate. If it matches the target time well enough, snap to it.
            if (!gnssfile.isEof() && fabs(gnss.time - next_target) < 0.5 * integration_length) {
                sow = gnss.time;
            } else {
                sow = next_target;
            }
        } else {
            sow += integration_length;
        }
    }

    gnssfile.close();

    auto te = std::chrono::steady_clock::now();
    std::cout << std::endl << std::endl << "Cost " << std::chrono::duration<double>(te - ts).count() << " s in total" << std::endl;

    // Run comparison script if enabled
    if (config["comparison"] && yamlToBool(config["comparison"]["enable"], false)) {
        try {
            std::string script_path = config["comparison"]["python_script"].as<std::string>();
            std::string truth_path = config["comparison"]["truth_file"].as<std::string>();
            
            std::filesystem::path nav_path(outputpath);
            nav_path /= "OB_GINS_TXT.nav";
            
            if (std::filesystem::exists(script_path) && std::filesystem::exists(truth_path) && std::filesystem::exists(nav_path)) {
                std::cout << "\n[info] Running comparison script..." << std::endl;
                
                auto quote_if_needed = [](const std::string& path) {
                    if (path.find(' ') != std::string::npos) {
                        return "\"" + path + "\"";
                    }
                    return path;
                };
                std::string command = "python " + quote_if_needed(script_path) + " " + quote_if_needed(nav_path.string()) + " " + quote_if_needed(truth_path);
                
                // Add label if specified
                //if (config["comparison"]["label"]) {
                //    command += " --label " + quote_if_needed(config["comparison"]["label"].as<std::string>());
                //}

                std::cout << "[cmd] " << command << std::endl;
                int ret = std::system(command.c_str());
                if (ret == 0) {
                    std::cout << "[info] Comparison script finished successfully." << std::endl;
                } else {
                    std::cout << "[error] Comparison script exited with code " << ret << std::endl;
                }
            } else {
                if (!std::filesystem::exists(script_path)) std::cout << "[error] Python script not found: " << script_path << std::endl;
                if (!std::filesystem::exists(truth_path)) std::cout << "[error] Truth file not found: " << truth_path << std::endl;
                if (!std::filesystem::exists(nav_path)) std::cout << "[error] Navigation output file not found: " << nav_path.string() << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "[error] Failed to execute comparison script: " << e.what() << std::endl;
        }
    }

    return 0;
}
