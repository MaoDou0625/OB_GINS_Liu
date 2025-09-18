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

#include "src/fileio/filesaver.h"
#include "src/fileio/gnssfileloader.h"
#include "src/fileio/imufileloader.h"
#include "src/fileio/odofileloader.h"

#include "src/factors/gnss_factor.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_manifold.h"
#include "src/factors/odo_factor.h"
#include "src/factors/angles_prior_factor.h"
#include "src/factors/sodo_prior_factor.h"
#include "src/factors/nhc_factor.h"
#include "src/factors/diff_yaw_factor.h"
#include "src/factors/scalar_prior_factor.h"
#include "src/factors/gyro_share_factor.h"
#include "src/factors/att_share_factor.h"
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"
#include "src/preintegration/preintegration_wheel.h"

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <deque>
#include <iomanip>
#include <cmath>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <array>
#include <unordered_map>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);

// Helper container for optional left/right odometer streams
struct OdoSource {
    std::string name;
    std::unique_ptr<OdoFileLoader> loader;   // may be null if disabled
    std::deque<ODO> buffer;                  // measurements within window
    ODO curr{};                              // latest read sample
    Vector3d lever = Vector3d::Zero();       // lever arm (m)
    Vector3d base_angle = Vector3d::Zero();  // odo mounting angles (deg in YAML -> rad here)
    bool enabled = false;
};

// Unified IMU source (main or wheel)
struct ImuSource {
    std::string name;
    bool enabled = false;
    std::unique_ptr<ImuFileLoader> loader;
    std::deque<IMU> buffer; // buffered increments within window
    IMU curr{};             // latest read sample

    IntegrationState state; // independent state for this IMU
    std::deque<std::shared_ptr<PreintegrationBase>> preint; // independent preintegrations

    // Wheel extrinsics (sensor->body)
    Vector3d extrinsic_angle = Vector3d::Zero(); // rbw (rad)
    Vector3d lever = Vector3d::Zero();           // lever (m)
    bool is_wheel = false;

    // Share-factor params (optional)
    double gyro_sigma = 0.03;              // rad/s
    double huber_delta = 1.0;
    double rbw_prior_sigma_rad = 3.0 * M_PI / 180.0; // rad
    double att_sigma_rad = 2.0 * M_PI / 180.0;       // rad
    double att_huber_delta = 1.0;

    // Running attitude estimate for AttShareFactor
    Eigen::Quaterniond q_est = Eigen::Quaterniond::Identity();
    bool q_inited = false;
};

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";

    auto ts = absl::Now();

    // Load configuration from YAML (paths, processing window, noise models, options)
    // On parse error: print message and exit
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // Processing time window (seconds) and sliding-window length
    // windows: number of nodes; [starttime, endtime]: time span to process
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // Solver iteration settings
    // num_iterations used per solve within each window
    int num_iterations = config["num_iterations"].as<int>();

    // Enable GNSS outlier culling (chi-square reweighting across the window)
    // Do GNSS outlier culling
    bool is_outlier_culling = config["is_outlier_culling"].as<bool>();

    // Initialization priors for state (velocity, attitude, IMU biases)
    // Attitude in degrees (converted to radians)
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // Input/output paths and IMU file format
    // gnssfile/imufile/outputpath + imudatalen (columns) and imudatarate (Hz)
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();

    int imudatalen  = config["imudatalen"] ? config["imudatalen"].as<int>() : 7;    // default columns
    int imudatarate = config["imudatarate"] ? config["imudatarate"].as<int>() : 200; // default Hz

    ImuSource imu_main, imu_left, imu_right;
    imu_main.name  = "main";
    imu_left.name  = "left";
    imu_right.name = "right";
    imu_left.is_wheel = true;
    imu_right.is_wheel = true;

    auto load_imu_node = [&](const YAML::Node &node, ImuSource &dst) {
        if (!node) return;
        bool enabled = false;
        if (node["enable"]) enabled = node["enable"].as<bool>();
        else if (node["isuseimu"]) enabled = node["isuseimu"].as<bool>(); // backward compat
        if (!enabled) return;
        if (!node["file"]) { std::cout << "IMU('" << dst.name << "') enabled but missing file" << std::endl; return; }
        std::string path = node["file"].as<std::string>();
        int cols = 7; if (node["columns"]) cols = node["columns"].as<int>(); else if (node["datalen"]) cols = node["datalen"].as<int>();
        int rate = imudatarate; if (node["rate_hz"]) rate = node["rate_hz"].as<int>(); else if (node["rate"]) rate = node["rate"].as<int>();
        dst.loader = std::make_unique<ImuFileLoader>(path, cols, rate);
        if (!dst.loader->isOpen()) { std::cout << "Failed to open IMU('" << dst.name << "'): " << path << std::endl; dst.loader.reset(); return; }
        if (node["extrinsic_deg"]) { auto a = node["extrinsic_deg"].as<std::vector<double>>(); if (a.size()==3) dst.extrinsic_angle = Vector3d(a.data())*D2R; }
        else if (node["rbw_deg"]) { auto a = node["rbw_deg"].as<std::vector<double>>(); if (a.size()==3) dst.extrinsic_angle = Vector3d(a.data())*D2R; }
        if (node["lever_m"]) { auto lv = node["lever_m"].as<std::vector<double>>(); if (lv.size()==3) dst.lever = Vector3d(lv.data()); }
        if (dst.is_wheel) {
            if (node["gyro_share"]) { auto gs = node["gyro_share"]; if (gs["sigma_radps"]) dst.gyro_sigma = gs["sigma_radps"].as<double>(); if (gs["huber_delta"]) dst.huber_delta = gs["huber_delta"].as<double>(); if (gs["rbw_prior_sigma_deg"]) dst.rbw_prior_sigma_rad = gs["rbw_prior_sigma_deg"].as<double>()*D2R; }
            if (node["att_share"]) { auto as = node["att_share"]; if (as["sigma_deg"]) dst.att_sigma_rad = as["sigma_deg"].as<double>()*D2R; if (as["huber_delta"]) dst.att_huber_delta = as["huber_delta"].as<double>(); }
        }
        dst.enabled = true;
    };

    // new schema
    load_imu_node(config["imu_main"], imu_main);
    load_imu_node(config["wheel_imu_left"], imu_left);
    load_imu_node(config["wheel_imu_right"], imu_right);
    // backward compat: single wheel_imu
    if (!imu_left.enabled && config["wheel_imu"]) {
        std::cout << "[Warning] 'wheel_imu' is deprecated. Please use 'wheel_imu_left/right'." << std::endl;
        load_imu_node(config["wheel_imu"], imu_left);
        imu_left.name = "left"; imu_left.is_wheel = true;
    }

    std::vector<ImuSource*> active_imus; if (imu_main.enabled) active_imus.push_back(&imu_main); if (imu_left.enabled) active_imus.push_back(&imu_left); if (imu_right.enabled) active_imus.push_back(&imu_right);
    if (active_imus.empty()) { std::cout << "配置文件未启用任何 IMU（imu_main / wheel_imu_left / wheel_imu_right）。" << std::endl; return -1; }
    ImuSource *driver = imu_main.enabled? &imu_main : (imu_left.enabled? &imu_left : &imu_right);

    // Consider Earth's rotation in mechanization (Coriolis and Earth-rate effects)
    // Affects preintegration dynamics if true
    bool isearth = config["isearth"].as<bool>();

    GnssFileLoader gnssfile(gnsspath);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    if (!navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // Installation parameters: lever arms and body-to-vehicle mounting angles
    // antlever/odolever in meters; bodyangle in degrees
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;

    // IMU鍣０鍙傛暟
    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();

        // 优先从启用的 IMU 区块(imu_main / wheel_imu)下的 imumodel 读取；没有则回退到全局 imumodel；再没有则用默认值。
    YAML::Node imu_model;
    if (config["imumodel"]) {
        imu_model = config["imumodel"];
    }

    double arw_deg_per_hr      = 0.24;  // 默认与历史配置一致
    double gbstd_deg_per_hr    = 50.0;
    double vrw_mps_sqrt_hour   = 0.24;
    double abstd_mg            = 250.0;
    double corr_time_hours     = 1.0;

    auto loadImuModelValue = [&](const char *key, double &target) {
        if (!imu_model) return;
        YAML::Node v = imu_model[key];
        if (v && v.IsScalar()) {
            try { target = v.as<double>(); } catch (...) {}
        }
    };

    loadImuModelValue("arw", arw_deg_per_hr);
    loadImuModelValue("gbstd", gbstd_deg_per_hr);
    loadImuModelValue("vrw", vrw_mps_sqrt_hour);
    loadImuModelValue("abstd", abstd_mg);
    loadImuModelValue("corrtime", corr_time_hours);

    parameters->gyr_arw      = arw_deg_per_hr * D2R / 60.0;
    parameters->gyr_bias_std = gbstd_deg_per_hr * D2R / 3600.0;
    parameters->acc_vrw      = vrw_mps_sqrt_hour / 60.0;
    parameters->acc_bias_std = abstd_mg * 1.0e-5;
    parameters->corr_time    = corr_time_hours * 3600.0;

    // Override IMU noise parameters from YAML (supports both top-level and per-IMU blocks)
    // Ensures values come from YAML instead of hard-coded defaults when provided.
    {
        YAML::Node imu_model_fix;
        if (driver) {
            YAML::Node sel = config[driver==&imu_main?"imu_main":(driver==&imu_left?"wheel_imu_left":"wheel_imu_right")];
            if (sel && sel["imumodel"]) imu_model_fix = sel["imumodel"];
        }
        if ((!imu_model_fix || !imu_model_fix.IsDefined()) && config["imumodel"]) {
            imu_model_fix = config["imumodel"];
        }
        auto getv = [&](const char *key, double defv) -> double {
            if (imu_model_fix && imu_model_fix.IsDefined()) {
                YAML::Node v = imu_model_fix[key];
                if (v && v.IsScalar()) {
                    try { return v.as<double>(); } catch (...) {}
                }
            }
            return defv;
        };
        // Read YAML values with defaults and update params
        const double arw_dph      = getv("arw",     0.24);
        const double vrw_mps_hr   = getv("vrw",     0.24);
        const double gbstd_dph    = getv("gbstd",  50.0);
        const double abstd_mg     = getv("abstd", 250.0);
        const double corrtime_hr  = getv("corrtime", 1.0);

        parameters->gyr_arw      = arw_dph * D2R / 60.0;       // deg/sqrt(hr) -> rad/s/sqrt(s)
        parameters->acc_vrw      = vrw_mps_hr / 60.0;          // m/s/sqrt(hr) -> m/s/sqrt(s)
        parameters->gyr_bias_std = gbstd_dph * D2R / 3600.0;   // deg/hr -> rad/s
        parameters->acc_bias_std = abstd_mg * 1.0e-5;          // mGal -> m/s^2
        parameters->corr_time    = corrtime_hr * 3600.0;       // hr -> s
    }

    // Odometer base options (optional block). Provide safe defaults if missing.
    YAML::Node odo_root  = config["odometer"];
    bool isuseodo        = false;
    Vector3d odo_std_def(0.05, 0.05, 0.05);
    double odo_srw_def   = 100.0 * 1e-6; // 100 ppm
    if (odo_root && odo_root["isuseodo"]) {
        isuseodo = odo_root["isuseodo"].as<bool>();
    }
    if (odo_root && odo_root["std"]) {
        vec = odo_root["std"].as<std::vector<double>>();
        if (vec.size() == 3) odo_std_def = Vector3d(vec.data());
    }
    if (odo_root && odo_root["srw"]) {
        odo_srw_def = odo_root["srw"].as<double>() * 1e-6;
    }
    parameters->odo_std = odo_std_def;
    parameters->odo_srw = odo_srw_def;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;

    // Parse odometer sources: support legacy `odometer` and per-wheel `odometer_left/right`
    YAML::Node odo_cfg = odo_root; // may be null if odometer block absent
    std::vector<OdoSource> odo_sources;
    auto load_odo_node = [&](const YAML::Node &node, const std::string &tag) {
        OdoSource src;
        src.name = tag;
        if (!node || !node["isuseodo"] || !node["isuseodo"].as<bool>()) {
            src.enabled = false;
            return src;
        }
        if (!node["file"]) {
            std::cout << "Odometer(" << tag << ") enabled but no file specified in config" << std::endl;
            return src;
        }
        std::string path = node["file"].as<std::string>();
        int cols         = node["columns"] ? node["columns"].as<int>() : 2;
        src.loader       = std::make_unique<OdoFileLoader>(path, cols);
        if (!src.loader->isOpen()) {
            std::cout << "Failed to open odometer file: " << path << std::endl;
            src.loader.reset();
            return src;
        }
        // lever (fallback to odolever)
        src.lever = odolever;
        if (node["lever"]) {
            auto lv = node["lever"].as<std::vector<double>>();
            if (lv.size() == 3) src.lever = Vector3d(lv.data());
        }
        // odoangle (fallback to bodyangle)
        src.base_angle = bodyangle;
        if (node["odoangle"]) {
            auto ang = node["odoangle"].as<std::vector<double>>();
            if (ang.size() == 3) src.base_angle = Vector3d(ang.data()) * D2R;
        }
        src.enabled = true;
        return src;
    };
    // Legacy single odometer
    if (isuseodo) {
        auto src = load_odo_node(odo_cfg, "odometer");
        if (src.enabled) odo_sources.push_back(std::move(src));
    }
    // Per-wheel nodes (optional)
    if (config["odometer_left"]) {
        auto src = load_odo_node(config["odometer_left"], "left");
        if (src.enabled) odo_sources.push_back(std::move(src));
    }
    if (config["odometer_right"]) {
        auto src = load_odo_node(config["odometer_right"], "right");
        if (src.enabled) odo_sources.push_back(std::move(src));
    }
    bool has_odo_sources = !odo_sources.empty();

    // Wheel IMUs are parsed via unified ImuSource above

    // Global gates override (unified yaw/acc thresholds/scales)
    if (config["gates"]) {
        auto g = config["gates"];
        if (g["yaw_rate"]) {
            auto y = g["yaw_rate"];
            if (y["thr"])   {}
            if (y["scale"]) {}
        }
    }
    // ODO factor configuration (robust loss, adaptive thresholds, priors)
    double odo_huber_delta        = 1.0;
    double odo_low_speed_thresh   = 0.2; // m/s
    double odo_low_speed_scale    = 5.0;
    double odo_yaw_rate_thresh    = 0.5; // rad/s
    double odo_yaw_rate_scale     = 2.0;
    double odo_accel_thresh       = 3.0; // m/s^2
    double odo_accel_scale        = 2.0;
    double sodo_prior_sigma       = 2.0e-2; // scale prior std
    double rbw_prior_sigma_rad    = 3.0 * M_PI / 180.0; // 3 deg
    // NHC options
    bool use_nhc                  = true;
    double nhc_sigma              = 0.1;  // m/s
    double nhc_huber_delta        = 0.5;
    if (odo_cfg && odo_cfg["factor"]) {
        auto f = odo_cfg["factor"];
        if (f["huber_delta"])          odo_huber_delta      = f["huber_delta"].as<double>();
        if (f["low_speed_thresh"])     odo_low_speed_thresh = f["low_speed_thresh"].as<double>();
        if (f["low_speed_scale"])      odo_low_speed_scale  = f["low_speed_scale"].as<double>();
        if (f["yaw_rate_thresh"])      odo_yaw_rate_thresh  = f["yaw_rate_thresh"].as<double>();
        if (f["yaw_rate_scale"])       odo_yaw_rate_scale   = f["yaw_rate_scale"].as<double>();
        if (f["accel_thresh"])         odo_accel_thresh     = f["accel_thresh"].as<double>();
        if (f["accel_scale"])          odo_accel_scale      = f["accel_scale"].as<double>();
        if (f["sodo_prior_sigma"])     sodo_prior_sigma     = f["sodo_prior_sigma"].as<double>();
        if (f["rbw_prior_sigma_deg"])  rbw_prior_sigma_rad  = f["rbw_prior_sigma_deg"].as<double>() * D2R;
        if (f["use_nhc"])              use_nhc              = f["use_nhc"].as<bool>();
        if (f["nhc_sigma"])            nhc_sigma            = f["nhc_sigma"].as<double>();
        if (f["nhc_huber_delta"])      nhc_huber_delta      = f["nhc_huber_delta"].as<double>();
    }
    // Apply unified gates if present (overrides)
    if (config["gates"]) {
        auto g = config["gates"];
        if (g["yaw_rate"]) {
            auto y = g["yaw_rate"];
            if (y["thr"])   odo_yaw_rate_thresh = y["thr"].as<double>();
            if (y["scale"]) odo_yaw_rate_scale  = y["scale"].as<double>();
        }
        if (g["accel"]) {
            auto a = g["accel"];
            if (a["thr"])   odo_accel_thresh = a["thr"].as<double>();
            if (a["scale"]) odo_accel_scale  = a["scale"].as<double>();
        }
    }

    // Differential yaw-rate (Δω) factor configuration
    YAML::Node diffyaw_cfg          = config["diff_yaw"];
    bool use_diffyaw                = false;
    double diffyaw_baseline         = 1.5;   // axle baseline [m]
    double diffyaw_sigma            = 0.05;  // rad/s
    double diffyaw_huber_delta      = 1.0;
    double diffyaw_pair_tolerance_s = 0.01;  // pair tolerance between L/R time tags
    bool   diffyaw_use_shared_gates = true;  // reuse odo yaw/acc gating to scale sigma / thresholds
    bool   diffyaw_hard_gate        = false; // skip residual when exceeding thresholds
    double diffyaw_gate_yaw         = 1.0;   // rad/s (used if not using shared gates)
    double diffyaw_gate_acc         = 5.0;   // m/s^2 (used if not using shared gates)
    bool   diffyaw_log_stats        = false; // dump dyaw_stats.csv
    bool   diffyaw_estimate_baseline= false; // estimate b online
    double diffyaw_baseline_sigma   = 0.05;  // m (prior sigma)
    double diffyaw_freeze_duration  = 30.0;  // s
    std::string diffyaw_gate_mode   = "";   // optional: hard|scale|off
    if (diffyaw_cfg) {
        if (diffyaw_cfg["enable"])           use_diffyaw                = diffyaw_cfg["enable"].as<bool>();
        if (diffyaw_cfg["baseline_m"])       diffyaw_baseline           = diffyaw_cfg["baseline_m"].as<double>();
        if (diffyaw_cfg["sigma_radps"])      diffyaw_sigma              = diffyaw_cfg["sigma_radps"].as<double>();
        if (diffyaw_cfg["huber_delta"])      diffyaw_huber_delta        = diffyaw_cfg["huber_delta"].as<double>();
        if (diffyaw_cfg["pair_tolerance_s"]) diffyaw_pair_tolerance_s   = diffyaw_cfg["pair_tolerance_s"].as<double>();
        if (diffyaw_cfg["use_shared_odo_gates"]) diffyaw_use_shared_gates = diffyaw_cfg["use_shared_odo_gates"].as<bool>();
        if (diffyaw_cfg["hard_gate"])         diffyaw_hard_gate          = diffyaw_cfg["hard_gate"].as<bool>();
        if (diffyaw_cfg["max_yaw_rate_radps"]) diffyaw_gate_yaw          = diffyaw_cfg["max_yaw_rate_radps"].as<double>();
        if (diffyaw_cfg["max_accel_ms2"])      diffyaw_gate_acc          = diffyaw_cfg["max_accel_ms2"].as<double>();
        if (diffyaw_cfg["log_stats"])         diffyaw_log_stats          = diffyaw_cfg["log_stats"].as<bool>();
        if (diffyaw_cfg["estimate_baseline"]) diffyaw_estimate_baseline  = diffyaw_cfg["estimate_baseline"].as<bool>();
        if (diffyaw_cfg["baseline_sigma_m"])  diffyaw_baseline_sigma     = diffyaw_cfg["baseline_sigma_m"].as<double>();
        if (diffyaw_cfg["freeze_duration_s"]) diffyaw_freeze_duration    = diffyaw_cfg["freeze_duration_s"].as<double>();
        if (diffyaw_cfg["gate_mode"])         diffyaw_gate_mode          = diffyaw_cfg["gate_mode"].as<std::string>();
    }
    if (!diffyaw_gate_mode.empty()) {
        if (diffyaw_gate_mode == "hard") {
            diffyaw_hard_gate = true;
        } else if (diffyaw_gate_mode == "off") {
            diffyaw_hard_gate = false;
            diffyaw_use_shared_gates = false; // no scaling
        } else {
            // scale
            diffyaw_hard_gate = false;
            // if gates block present, scaling uses shared values
        }
    }
    // Baseline optimization parameter (shared across windows)
    double dyaw_baseline_param = diffyaw_baseline;

    // GNSS浠跨湡涓柇閰嶇疆
    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // Data alignment: advance IMU and GNSS streams to starttime
    // For each active IMU, read until first sample >= starttime
    for (auto *s : active_imus) {
        if (!s->loader) continue;
        s->curr = s->loader->next();
        IMU last{};
        while (s->curr.time < starttime && !s->loader->isEof()) {
            last = s->curr; s->curr = s->loader->next();
        }
        if (last.time > 0) s->buffer.push_back(last);
    }

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    // Align ODO stream to start time
    std::deque<ODO> odolist;
    if (has_odo_sources) {
        for (auto &src : odo_sources) {
            if (!src.loader) continue;
            src.curr = src.loader->next();
            while (src.curr.time < starttime && !src.loader->isEof()) {
                src.curr = src.loader->next();
            }
        }
    }
    // Wheel IMUs are aligned via unified ImuSource loop above

    // Initialize station origin (geodetic) and convert GNSS to local frame
    Vector3d station_origin = gnss.blh;
    parameters->gravity     = Earth::gravity(gnss.blh);
    gnss.blh                = Earth::global2local(station_origin, gnss.blh);

    // Save station origin in parameters (used by Earth model computations)
    parameters->station = station_origin;

    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::deque<std::shared_ptr<PreintegrationBase>> preintegrationlist;
    std::deque<GNSS> gnsslist;
    std::deque<double> timelist;
    std::vector<Vector3d> omega_nodes;
    std::deque<IMU> imuqueue; // keep recent IMU for precise omega/acc lookup

    // Decouple wheel speed from IMU preintegration; use IMU-only preintegration here
    Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(false, isearth);

    // Initial state at first integer-second GNSS (position, attitude, velocity, biases)
    // initialization
    IntegrationState state_curr = {
        .time = round(gnss.time),
        .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = initvel,
        .bg   = initbg,
        .ba   = initba,
        .sodo = 0.0,
        .abv  = {bodyangle[1], bodyangle[2]},
    };
    std::cout << "Initilization at " << gnss.time << " s " << std::endl;

    // Initialize each IMU's independent state
    for (auto *s : active_imus) s->state = state_curr;

    statelist[0]     = state_curr;
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // Initial preintegration: seed with driver IMU sample and initial state
    IMU drv_seed = driver->buffer.empty()? driver->curr : driver->buffer.back();
    preintegrationlist.emplace_back(Preintegration::createPreintegration(parameters, drv_seed, state_curr, preintegration_options));
    // Initialize each IMU's independent state
    for (auto *s : active_imus) s->state = state_curr;
    // Seed wheel IMU preintegrations for independence
    for (auto *s : active_imus) {
        if (s==driver) continue;
        IMU seed = s->buffer.empty()? s->curr : s->buffer.back();
        s->preint.clear();
        s->preint.emplace_back(std::make_shared<PreintegrationWheel>(parameters, seed, s->state, s->extrinsic_angle, s->lever));
    }

    // Read next GNSS epoch for subsequent steps
    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh            = Earth::global2local(station_origin, gnss.blh);

    // Marginalization bookkeeping (previous prior and parameter blocks)
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    // Move to next integration node (advance target time by INTEGRATION_LENGTH)
    sow += INTEGRATION_LENGTH;

    // Prepare driver IMU iteration variables
    IMU imu_pre = drv_seed;
    IMU imu_cur = driver->curr;

    while (true) {
        if ((imu_cur.time > endtime) || (driver->loader && driver->loader->isEof())) {
            break;
        }

        // Add new IMU sample to ongoing preintegration
        // Add new imu data to preintegration
        preintegrationlist.back()->addNewImu(imu_cur);
        // Append to IMU queue for precise omega/acc extraction
        imuqueue.push_back(imu_cur);
        // Drop old IMU beyond window horizon
        while (!imuqueue.empty() && imuqueue.front().time < timelist.front() - 2.0 * INTEGRATION_LENGTH) {
            imuqueue.pop_front();
        }

        // No additional high-rate write here; nodes are now at INTEGRATION_LENGTH (e.g., 0.01s)

        // Advance secondary IMUs up to driver's current time
        for (auto *s : active_imus) {
            if (s==driver || !s->loader) continue;
            while (!s->loader->isEof() && s->curr.time <= imu_cur.time) {
                s->buffer.push_back(s->curr);
                if (!s->preint.empty()) s->preint.back()->addNewImu(s->curr);
                s->curr = s->loader->next();
            }
            while (!s->buffer.empty() && s->buffer.front().time < timelist.front() - 0.5 * INTEGRATION_LENGTH) {
                s->buffer.pop_front();
            }
        }

        imu_pre = imu_cur;
        imu_cur = driver->loader->next();

        if (imu_cur.time > sow) {
            // On GNSS epoch: add GNSS measurement and fetch next fix
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
                gnsslist.push_back(gnss);

                gnss = gnssfile.next();
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold)) {
                    gnss = gnssfile.next();
                }

                // GNSS outage emulation: skip GNSS fixes during configured intervals
                // do GNSS outage
                if (isuseoutage) {
                    if (lround(gnss.time) == outagetime) {
                        std::cout << "GNSS outage at " << outagetime << " s" << std::endl;
                        for (int k = 0; k < outagelen; k++) {
                            gnss = gnssfile.next();
                        }
                        outagetime += outageperiod;
                    }
                }

                parameters->gravity = Earth::gravity(gnss.blh);
                gnss.blh            = Earth::global2local(station_origin, gnss.blh);
                if (gnssfile.isEof()) {
                    gnss.time = 0;
                }
            }

            // IMU interpolation at exact boundary when needed
            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) {
            } else if (isneed == 1) {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = driver->loader->next();
            } else if (isneed == 2) {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }

            // Next integration node (integer-second boundary reached)
            // next time node
            timelist.push_back(sow);
            sow += INTEGRATION_LENGTH;

            // Push current integer-second state into sliding window buffers
            state_curr                               = preintegrationlist.back()->currentState();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Preintegration::stateToData(state_curr, preintegration_options);
            // Record angular velocity at this node (approximate)
            Vector3d omega_at_node = imu_pre.dtheta / std::max(imu_pre.dt, 1e-6);
            omega_nodes.push_back(omega_at_node);

            // Build nonlinear least-squares problem for the current window
            // construct optimization problem
            {
                ceres::Problem::Options problem_options;
                problem_options.enable_fast_removal = true;

                ceres::Problem problem(problem_options);
                ceres::Solver solver;
                ceres::Solver::Summary summary;
                ceres::Solver::Options options;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
                options.num_threads                = 4;

                // Add parameter blocks for each node in the window
                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    // Pose (position + quaternion orientation) manifold parameterization
                    ceres::Manifold *manifold = new PoseManifold();
                    problem.AddParameterBlock(statedatalist[k].pose, Preintegration::numPoseParameter(), manifold);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Preintegration::numMixParameter(preintegration_options));
                }

                // GNSS measurement factors (position with robust loss)
                // GNSS factors
                int index = 0;

                ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
                std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id;
                for (const auto &gnss : gnsslist) {
                    auto factor = new GnssFactor(gnss, antlever);
                    for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                        if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                            auto id = problem.AddResidualBlock(factor, loss_function, statedatalist[i].pose);
                            gnss_residualblock_id.push_back(std::make_pair(gnss.time, id));
                            index++;
                            break;
                        }
                    }
                }

                // IMU preintegration factors between consecutive states
                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                {
                    // IMU bias constraint (random-walk process prior)
                    // add IMU bias-constraint factors
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // Add wheel-speed (odometer) factors as separate measurement streams
                if (has_odo_sources) {
                    // Advance ODO buffer up to the latest node time
                    {
                        // Add shared parameters (scale and rbw) once
                        const int mix_dim = Preintegration::numMixParameter(preintegration_options);
                        ceres::LossFunction *odo_loss = new ceres::HuberLoss(odo_huber_delta);

                        static double sodo_param = 1.0; // global scale across sources
                        problem.AddParameterBlock(&sodo_param, 1);
                        problem.AddResidualBlock(new SodoPriorFactor(1.0, sodo_prior_sigma), nullptr, &sodo_param);

                        static double rbw_param[3] = {0.0, 0.0, 0.0};
                        problem.AddParameterBlock(rbw_param, 3);
                        problem.AddResidualBlock(new AnglesPriorFactor(rbw_prior_sigma_rad), nullptr, rbw_param);

                        for (auto &src : odo_sources) {
                            if (!src.loader) continue;
                            // Read forward to the latest node time
                            while (!src.loader->isEof() && src.curr.time <= *timelist.rbegin()) {
                                src.buffer.push_back(src.curr);
                                src.curr = src.loader->next();
                            }
                            // Drop outdated
                            while (!src.buffer.empty() && src.buffer.front().time < timelist.front() - 0.5 * INTEGRATION_LENGTH) {
                                src.buffer.pop_front();
                            }
                            // Build residuals for this source
                            for (const auto &m : src.buffer) {
                                if (m.time < timelist.front() - MINIMUM_INTERVAL || m.time > timelist.back() + MINIMUM_INTERVAL) {
                                    continue;
                                }
                                // Find nearest node index
                                size_t nearest = 0; double best = 1e9;
                                for (size_t i = 0; i < timelist.size(); ++i) {
                                    double d = fabs(m.time - timelist[i]);
                                    if (d < best) { best = d; nearest = i; }
                                }
                                // Approximate omega/acc at measurement time
                                Vector3d omega_b = Vector3d::Zero();
                                Vector3d acc_b   = Vector3d::Zero();
                                for (size_t ii = 1; ii < imuqueue.size(); ++ii) {
                                    if (imuqueue[ii - 1].time <= m.time && imuqueue[ii].time >= m.time) {
                                        double dt  = std::max(imuqueue[ii].dt, 1e-6);
                                        omega_b    = imuqueue[ii].dtheta / dt;
                                        acc_b      = imuqueue[ii].dvel / dt;
                                        break;
                                    }
                                }
                                if (omega_b.isZero(0)) {
                                    omega_b = (omega_nodes.size() > nearest) ? omega_nodes[nearest] : Vector3d::Zero();
                                }
                                // Adaptive sigma
                                double sigma = parameters->odo_std[0];
                                if (fabs(m.vel) < odo_low_speed_thresh) sigma *= odo_low_speed_scale;
                                if (fabs(omega_b.z()) > odo_yaw_rate_thresh) sigma *= odo_yaw_rate_scale;
                                if (acc_b.norm() > odo_accel_thresh) sigma *= odo_accel_scale;

                                Vector3d omega_x_l = omega_b.cross(src.lever);
                                auto factor        = new OdoFactor(m.vel, sigma, omega_x_l, mix_dim, src.base_angle);
                                problem.AddResidualBlock(factor, odo_loss,
                                                         statedatalist[nearest].pose,
                                                         statedatalist[nearest].mix,
                                                         &sodo_param,
                                                         rbw_param);
                            }
                        }

                        // Differential yaw-rate residuals (need both left/right wheel speeds)
                        if (use_diffyaw) {
                            ceres::LossFunction *dy_loss = new ceres::HuberLoss(diffyaw_huber_delta);
                            // Find left/right sources
                            OdoSource *left_src = nullptr; OdoSource *right_src = nullptr;
                            for (auto &s : odo_sources) {
                                if (!s.loader) continue;
                                if (s.name == "left") left_src = &s;
                                if (s.name == "right") right_src = &s;
                            }
                            if (left_src && right_src) {
                                // Stats counters
                                size_t pairs_total = 0, kept_cnt = 0, scaled_cnt = 0, gated_cnt = 0;

                                // Two-pointer pairing within tolerance
                                size_t li = 0, ri = 0;
                                const auto &LB = left_src->buffer;
                                const auto &RB = right_src->buffer;
                                auto interp_odo = [&](const std::deque<ODO>& B, size_t idx, double t) -> double {
                                    if (B.empty()) return 0.0;
                                    size_t i0 = std::min(idx, B.size()-1);
                                    size_t i1 = std::min(i0+1, B.size()-1);
                                    const double t0 = B[i0].time;
                                    const double v0 = B[i0].vel;
                                    if (i1 == i0) return v0;
                                    const double t1 = B[i1].time;
                                    const double v1 = B[i1].vel;
                                    if (t1 <= t0) return v0;
                                    double a = std::clamp((t - t0)/(t1 - t0), 0.0, 1.0);
                                    return v0 + a * (v1 - v0);
                                };
                                while (li < LB.size() && ri < RB.size()) {
                                    double tl = LB[li].time, tr = RB[ri].time;
                                    if (tl < timelist.front() - MINIMUM_INTERVAL) { ++li; continue; }
                                    if (tr < timelist.front() - MINIMUM_INTERVAL) { ++ri; continue; }
                                    if (tl > timelist.back() + MINIMUM_INTERVAL || tr > timelist.back() + MINIMUM_INTERVAL) break;

                                    double dt_pair = fabs(tl - tr);
                                    if (dt_pair <= diffyaw_pair_tolerance_s) {
                                        pairs_total++;
                                        const double t = 0.5 * (tl + tr);
                                        const double vL = interp_odo(LB, li, t);
                                        const double vR = interp_odo(RB, ri, t);
                                        const double omega_wheel = (vL - vR) / std::max(diffyaw_baseline, 1e-6);

                                        // Find nearest graph node
                                        size_t nearest = 0; double best = 1e9;
                                        for (size_t i = 0; i < timelist.size(); ++i) {
                                            double d = fabs(t - timelist[i]);
                                            if (d < best) { best = d; nearest = i; }
                                        }
                                        // Estimate omega_z and accel magnitude at time t
                                        Vector3d omega_b = Vector3d::Zero();
                                        Vector3d acc_b   = Vector3d::Zero();
                                        for (size_t ii = 1; ii < imuqueue.size(); ++ii) {
                                            if (imuqueue[ii - 1].time <= t && imuqueue[ii].time >= t) {
                                                double dt  = std::max(imuqueue[ii].dt, 1e-6);
                                                omega_b    = imuqueue[ii].dtheta / dt;
                                                acc_b      = imuqueue[ii].dvel / dt;
                                                break;
                                            }
                                        }
                                        if (omega_b.isZero(0)) {
                                            omega_b = (omega_nodes.size() > nearest) ? omega_nodes[nearest] : Vector3d::Zero();
                                        }

                                        // Hard gate or adaptive scaling
                                        const double abs_yaw = fabs(omega_b.z());
                                        const double accmag  = acc_b.norm();
                                        double sigma = diffyaw_sigma;
                                        if (diffyaw_hard_gate) {
                                            double yaw_thr = diffyaw_use_shared_gates ? odo_yaw_rate_thresh : diffyaw_gate_yaw;
                                            double acc_thr = diffyaw_use_shared_gates ? odo_accel_thresh     : diffyaw_gate_acc;
                                            if (abs_yaw > yaw_thr || accmag > acc_thr) {
                                                // skip this residual
                                                gated_cnt++;
                                                ++li; ++ri;
                                                continue;
                                            }
                                        } else if (diffyaw_use_shared_gates) {
                                            if (abs_yaw > odo_yaw_rate_thresh) sigma *= odo_yaw_rate_scale;
                                            if (accmag  > odo_accel_thresh)    sigma *= odo_accel_scale;
                                            if (sigma != diffyaw_sigma) scaled_cnt++;
                                        }

                                        if (diffyaw_estimate_baseline) {
                                            problem.AddParameterBlock(&dyaw_baseline_param, 1);
                                            // Freeze for initial duration
                                            bool freeze_now = (timelist.back() - timelist.front()) < diffyaw_freeze_duration;
                                            if (freeze_now) problem.SetParameterBlockConstant(&dyaw_baseline_param);
                                            // Prior on baseline
                                            problem.AddResidualBlock(new ScalarPriorFactor(diffyaw_baseline, diffyaw_baseline_sigma), nullptr, &dyaw_baseline_param);

                                            auto dy = new DiffYawFactor(omega_b.z(), vL, vR, sigma, mix_dim, true);
                                            problem.AddResidualBlock(dy, dy_loss, statedatalist[nearest].mix, &dyaw_baseline_param);
                                        } else {
                                            auto dy = new DiffYawFactor(omega_b.z(), omega_wheel, sigma, mix_dim);
                                            problem.AddResidualBlock(dy, dy_loss, statedatalist[nearest].mix);
                                        }
                                        kept_cnt++;

                                        ++li; ++ri;
                                    } else if (tl < tr) {
                                        ++li;
                                    } else {
                                        ++ri;
                                    }
                                }

                                if (diffyaw_log_stats) {
                                    static bool header = false;
                                    std::string path = outputpath + "/dyaw_stats.csv";
                                    std::ofstream ofs(path, std::ios::app);
                                    if (ofs) {
                                        if (!header) {
                                            ofs << "time_end,pairs,kept,scaled,gated,gate_mode,estimate_b,freeze_s\n";
                                            header = true;
                                        }
                                        std::string mode = diffyaw_hard_gate ? "hard" : (diffyaw_use_shared_gates ? "scale" : "off");
                                        double win_len = timelist.back() - timelist.front();
                                        ofs << timelist.back() << "," << pairs_total << "," << kept_cnt << "," << scaled_cnt << "," << gated_cnt << "," << mode << "," << (diffyaw_estimate_baseline?1:0) << "," << win_len << "\n";
                                    }
                                    // Also dump baseline snapshot (pre-solve for current window)
                                    if (diffyaw_estimate_baseline) {
                                        std::string pathb = outputpath + "/dyaw_baseline.csv";
                                        std::ofstream ofb(pathb, std::ios::app);
                                        if (ofb) {
                                            static bool header_b2 = false;
                                            if (!header_b2) {
                                                ofb << "time_end,baseline,freeze\n";
                                                header_b2 = true;
                                            }
                                            bool freeze_now = (timelist.back() - timelist.front()) < diffyaw_freeze_duration;
                                            ofb << timelist.back() << "," << dyaw_baseline_param << "," << (freeze_now?1:0) << "\n";
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Add NHC factors at each node (soft constraint on lateral/vertical velocity)
                if (use_nhc) {
                    ceres::LossFunction *nhc_loss = new ceres::HuberLoss(nhc_huber_delta);
                    const int mix_dim             = Preintegration::numMixParameter(preintegration_options);
                    for (size_t i = 0; i < timelist.size(); ++i) {
                        Vector3d omega_b = (omega_nodes.size() > i) ? omega_nodes[i] : Vector3d::Zero();
                        // approximate acc near node time
                        Vector3d acc_b = Vector3d::Zero();
                        for (size_t ii = 1; ii < imuqueue.size(); ++ii) {
                            if (imuqueue[ii - 1].time <= timelist[i] && imuqueue[ii].time >= timelist[i]) {
                                double dt = std::max(imuqueue[ii].dt, 1e-6);
                                acc_b     = imuqueue[ii].dvel / dt;
                                break;
                            }
                        }
                        double sigma = nhc_sigma;
                        if (fabs(omega_b.z()) > odo_yaw_rate_thresh) sigma *= odo_yaw_rate_scale;
                        if (acc_b.norm() > odo_accel_thresh) sigma *= odo_accel_scale;

                        auto nhc = new NhcFactor(sigma, mix_dim);
                        problem.AddResidualBlock(nhc, nhc_loss, statedatalist[i].pose, statedatalist[i].mix);
                    }
                }

                // Marginalization prior factor from previous window (if valid)
                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                // Solve least-squares (Levenberg-Marquardt; sparse normal Cholesky)
                // solve the Least-Squares problem
                options.max_num_iterations = num_iterations / 4;
                solver.Solve(options, &problem, &summary);

                // Log baseline convergence if enabled
                if (use_diffyaw && diffyaw_estimate_baseline && diffyaw_log_stats) {
                    std::string pathb = outputpath + "/dyaw_baseline.csv";
                    std::ofstream ofb(pathb, std::ios::app);
                    if (ofb) {
                        static bool header_b = false;
                        if (!header_b) {
                            ofb << "time_end,baseline,freeze\n";
                            header_b = true;
                        }
                        bool freeze_now = (timelist.back() - timelist.front()) < diffyaw_freeze_duration;
                        ofb << *timelist.rbegin() << "," << dyaw_baseline_param << "," << (freeze_now?1:0) << "\n";
                    }
                }

                // Wheel-IMU gyro/att share factors via unified IMU sources
                {
                    const int mix_dim = Preintegration::numMixParameter(preintegration_options);
                    for (auto *sp : active_imus) {
                        if (!sp->is_wheel || !sp->loader) continue;
                        // small-angle extrinsic delta with prior
                        static std::unordered_map<std::string, std::array<double,3>> rbw_param_map;
                        if (!rbw_param_map.count(sp->name)) rbw_param_map[sp->name] = {0.0,0.0,0.0};
                        auto &rbw_param = rbw_param_map[sp->name];
                        problem.AddParameterBlock(rbw_param.data(), 3);
                        problem.AddResidualBlock(new AnglesPriorFactor(sp->rbw_prior_sigma_rad), nullptr, rbw_param.data());

                        ceres::LossFunction *gs_loss = new ceres::HuberLoss(sp->huber_delta);
                        ceres::LossFunction *as_loss = new ceres::HuberLoss(sp->att_huber_delta);
                        for (const auto &m : sp->buffer) {
                            if (m.time < timelist.front() - MINIMUM_INTERVAL || m.time > timelist.back() + MINIMUM_INTERVAL) continue;
                            // nearest node index
                            size_t nearest = 0; double best = 1e9;
                            for (size_t i = 0; i < timelist.size(); ++i) { double d = fabs(m.time - timelist[i]); if (d < best) { best = d; nearest = i; } }
                            // omega_body at m.time from driver IMU
                            Vector3d omega_b = Vector3d::Zero();
                            for (size_t ii = 1; ii < imuqueue.size(); ++ii) {
                                if (imuqueue[ii - 1].time <= m.time && imuqueue[ii].time >= m.time) { double dt = std::max(imuqueue[ii].dt, 1e-6); omega_b = imuqueue[ii].dtheta / dt; break; }
                            }
                            if (omega_b.isZero(0)) omega_b = (omega_nodes.size()>nearest)? omega_nodes[nearest]:Vector3d::Zero();
                            // wheel omega
                            double dtw = std::max(m.dt, 1e-6);
                            Vector3d omega_w = m.dtheta / dtw;
                            // gating
                            Vector3d acc_b = Vector3d::Zero();
                            for (size_t ii = 1; ii < imuqueue.size(); ++ii) { if (imuqueue[ii - 1].time <= m.time && imuqueue[ii].time >= m.time) { double dt = std::max(imuqueue[ii].dt, 1e-6); acc_b = imuqueue[ii].dvel / dt; break; } }
                            double sigma = sp->gyro_sigma;
                            if (fabs(omega_b.z()) > odo_yaw_rate_thresh) sigma *= odo_yaw_rate_scale;
                            if (acc_b.norm() > odo_accel_thresh) sigma *= odo_accel_scale;

                            auto f = new GyroShareFactor(omega_b, omega_w, sp->extrinsic_angle, sigma, mix_dim);
                            problem.AddResidualBlock(f, gs_loss, statedatalist[nearest].mix, rbw_param.data());

                            if (!sp->q_inited) {
                                Eigen::Quaterniond q_body(statedatalist[nearest].pose[6], statedatalist[nearest].pose[3], statedatalist[nearest].pose[4], statedatalist[nearest].pose[5]);
                                Eigen::Matrix3d Rbw = Rotation::euler2matrix(sp->extrinsic_angle); Eigen::Matrix3d Rwb = Rbw.transpose();
                                sp->q_est = Rotation::matrix2quaternion(Rwb * q_body.toRotationMatrix()); sp->q_inited = true;
                            }
                            Eigen::Quaterniond dq = Rotation::rotvec2quaternion(m.dtheta); sp->q_est = sp->q_est * dq;
                            auto as = new AttShareFactor(sp->q_est, sp->extrinsic_angle, sp->att_sigma_rad);
                            problem.AddResidualBlock(as, as_loss, statedatalist[nearest].pose, rbw_param.data());
                        }
                    }
                }

                // TODO: Just a example, you need remodify.
                // Do GNSS outlier culling using chi-square test
                if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                    // 3 degrees of freedom, 0.05
                    double chi2_threshold = 7.815;

                    // Find GNSS outliers in the window
                    std::unordered_set<double> gnss_outlier;
                    for (size_t k = 0; k < gnsslist.size(); k++) {
                        auto time = gnss_residualblock_id[k].first;
                        auto id   = gnss_residualblock_id[k].second;

                        double cost;
                        double chi2;

                        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
                        chi2 = cost * 2;

                        if (chi2 > chi2_threshold) {
                            gnss_outlier.insert(time);

                            // Reweigthed GNSS
                            double scale = sqrt(chi2 / chi2_threshold);
                            gnsslist[k].std *= scale;
                        }
                    }
                    // // Log outliers
                    // if (!gnss_outlier.empty()) {
                    //     std::string log = absl::StrFormat("Reweight GNSS outlier at %g:", sow - 1);
                    //     for (const auto& time:gnss_outlier) {
                    //         absl::StrAppendFormat(&log, " %g", time);
                    //     }
                    //     std::cout << log << std::endl;
                    // }

                    // Remove all old GNSS factors
                    for (const auto &block : gnss_residualblock_id) {
                        problem.RemoveResidualBlock(block.second);
                    }

                    // Add GNSS factors without loss function
                    index = 0;
                    for (auto &gnss : gnsslist) {
                        auto factor = new GnssFactor(gnss, antlever);
                        for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                            if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                                problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);
                                index++;
                                break;
                            }
                        }
                    }
                }

                options.max_num_iterations = num_iterations * 3 / 4;
                solver.Solve(options, &problem, &summary);

                // Print processing progress percentage
                // output the percentage
                int percent            = ((int) sow - starttime) * 100 / (endtime - starttime);
                static int lastpercent = 0;
                if (abs(percent - lastpercent) >= 1) {
                    lastpercent = percent;
                    std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                    flush(std::cout);
                }
            }

            if (preintegrationlist.size() == static_cast<size_t>(windows)) {
                {
                    // Marginalization: construct new prior and shift window
                    // marginalization
                    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                    if (last_marginalization_info && last_marginalization_info->isValid()) {

                        std::vector<int> marginilized_index;
                        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                            if (last_marginalization_parameter_blocks[k] == statedatalist[0].pose ||
                                last_marginalization_parameter_blocks[k] == statedatalist[0].mix) {
                                marginilized_index.push_back(static_cast<int>(k));
                            }
                        }

                        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // IMU preintegration residual to be marginalized
                    // preintegration factors
                    {
                        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS residual to be marginalized (if aligned)
                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    // Perform marginalization to update the prior
                    // do marginalization
                    marginalization_info->marginalization();

                    // Remap parameter block pointers for the new prior
                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                }

                // Sliding window update: drop oldest node and shift arrays
                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time)) {
                        gnsslist.pop_front();
                    }
                    timelist.pop_front();
                    preintegrationlist.pop_front();
                    if (!omega_nodes.empty()) {
                        omega_nodes.erase(omega_nodes.begin());
                    }

                    for (int k = 0; k < windows; k++) {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k]     = Preintegration::stateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Preintegration::stateFromData(statedatalist[windows], preintegration_options);
                    state_curr         = statelist[windows];
                }
            } else {
                state_curr =
                    Preintegration::stateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            // write result
            writeNavResult(*timelist.rbegin(), station_origin, state_curr, navfile, errfile);

            // Start a new preintegration segment from the latest state
            // build a new preintegration object
            preintegrationlist.emplace_back(
                Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
            // also start new segments for secondary IMUs
            for (auto *s : active_imus) {
                if (s==driver) continue;
                s->state = Preintegration::stateFromData(statedatalist[preintegrationlist.size()-1], preintegration_options);
                IMU seed = s->curr;
                s->preint.emplace_back(std::make_shared<PreintegrationWheel>(parameters, seed, s->state, s->extrinsic_angle, s->lever));
            }
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    // Close IMU files
    for (auto *s : active_imus) { if (s->loader) s->loader->close(); }
    gnssfile.close();

    auto te = absl::Now();
    std::cout << std::endl << std::endl << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

    return 0;
}

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile) {
    vector<double> result;

    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // Close to the first epoch: treat as previous segment (no split)
        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // Close to the second epoch: include current IMU sample fully
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        // Otherwise, interpolate and split the IMU sample at the target time
        // need interpolation
        return 2;
    }

    return 0;
}




