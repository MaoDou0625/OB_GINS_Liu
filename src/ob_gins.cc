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

#include "src/factors/gnss_factor.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_manifold.h"
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"
// Adapter to unify main-IMU and wheel-IMU dispatch
#include "src/bridge/imu_adapter.h"
// Wheel-IMU state for wheel-specific output
#include "src/wheel/integration_state_wheel.h"
// (Wheel preintegration headers were included temporarily for wiring; removed per request.)

#include <chrono>
#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <unordered_set>
#include <cstdint>

// Debug utilities
#include "src/common/debug.h"
#include "src/common/logging.h"

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);
// Wheel-only writer to allow saving dual navigation results later
void writeNavResultWheel(double time, const Vector3d &origin, const WheelIntegrationState &state, FileSaver &navfile,
                         FileSaver &errfile);

// YAML 安全读取：宽松解析布尔量（支持 true/false/1/0/yes/no/on/off，字符串或数值均可）

static bool yamlToBool(const YAML::Node &node, bool def = false) {
    try {
        if (!node || node.IsNull()) return def;
        if (node.IsScalar()) {
            try {
                // try string parse
                std::string t = node.as<std::string>();
                std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return (char) std::tolower(c); });
                if (t == "true" || t == "1" || t == "yes" || t == "on") return true;
                if (t == "false" || t == "0" || t == "no" || t == "off") return false;
            } catch (...) {
                // 蹇界暐锛岃繘鍏ユ暟鍊?甯冨皵鍥為€€
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

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";
    // 初始化 glog（终端彩色输出，默认不写文件）
    Logging::initialization(argv, /*logtostderr=*/true, /*logtofile=*/false);

    auto ts = std::chrono::steady_clock::now();

    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // processing time
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // keyframe interval (sec). Default 1.0; allow 20鈥?00 Hz via config
    double integration_length = INTEGRATION_LENGTH;
    if (config["kf_interval_sec"]) {
        try { integration_length = config["kf_interval_sec"].as<double>(); } catch (...) {}
    }

    // number of iterations
    int num_iterations = config["num_iterations"].as<int>();

    // Do GNSS outlier culling (fallback false)
    bool is_outlier_culling = yamlToBool(config["is_outlier_culling"], false);
    // save multi-IMU results (main/left/right) in parallel
    bool save_multi_imu     = yamlToBool(config["save_multi_imu"], false);

    // Debug controls (optional)
    bool dbg_enable = false;
    int  dbg_level  = 0;
    bool dbg_dual_chain = false; // run wheel and main preintegration in parallel for comparison
    if (config["debug"]) {
        auto dbg = config["debug"];
        dbg_enable = yamlToBool(dbg["enable"], false);
        if (dbg["level"])  dbg_level  = dbg["level"].as<int>();
        dbg_dual_chain = yamlToBool(dbg["dual_chain"], false);
    }
    Debug::set(dbg_enable, dbg_level);

    // initialization（支持按链分别定义初始姿态，默认继承全局 initatt）
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;
    // per-chain attitude overrides
    auto read_initatt_for = [&](const YAML::Node &node, const Vector3d &fallback) -> Vector3d {
        Vector3d att = fallback;
        try {
            if (node) {
                if (node["initatt"]) {
                    auto v = node["initatt"].as<std::vector<double>>();
                    if (v.size() >= 3) { Vector3d tmp(v.data()); att = tmp * D2R; }
                } else if (node["init_yaw"]) {
                    double yaw_deg = node["init_yaw"].as<double>();
                    att = Vector3d(0, 0, yaw_deg * D2R);
                }
            }
        } catch (...) { /* keep fallback */ }
        return att;
    };
    Vector3d initatt_main  = read_initatt_for(imu_node_main,  initatt);
    Vector3d initatt_left  = read_initatt_for(imu_node_left,  initatt);
    Vector3d initatt_right = read_initatt_for(imu_node_right, initatt);

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // data file
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string imupath;
    int imudatalen  = 0;
    int imudatarate = 0;
    std::string outputpath = config["outputpath"].as<std::string>();

    bool use_main = false, use_wheel_left = false, use_wheel_right = false;
    if (config["run_mode"]) {
        auto rm = config["run_mode"];
        use_main        = yamlToBool(rm["imu_main_enable"], use_main);
        use_wheel_left  = yamlToBool(rm["wheel_left_enable"], use_wheel_left);
        use_wheel_right = yamlToBool(rm["wheel_right_enable"], use_wheel_right);
    }

    YAML::Node imu_node;
    YAML::Node imu_node_main, imu_node_left, imu_node_right;
    if (config["imu_main"])       imu_node_main  = config["imu_main"];
    if (config["imu_wheel_left"]) imu_node_left  = config["imu_wheel_left"];
    if (config["imu_wheel_right"])imu_node_right = config["imu_wheel_right"];
    if (use_main && config["imu_main"]) {
        imu_node = config["imu_main"];
    } else if (use_wheel_left && config["imu_wheel_left"]) {
        imu_node = config["imu_wheel_left"];
    } else if (use_wheel_right && config["imu_wheel_right"]) {
        imu_node = config["imu_wheel_right"];
    }

    if (imu_node) {
        if (imu_node["file"])      imupath     = imu_node["file"].as<std::string>();
        if (imu_node["columns"])   imudatalen  = imu_node["columns"].as<int>();
        if (imu_node["rate_hz"])   imudatarate = imu_node["rate_hz"].as<int>();
        // 鍏煎鏃х増鏈€?
        if (config["imufile"])     imupath     = config["imufile"].as<std::string>();
        if (config["imudatalen"])  imudatalen  = config["imudatalen"].as<int>();
        if (config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();
    }
    DBG_LOG(1, "CFG", "run_mode main=" << use_main << ", wheel_left=" << use_wheel_left
            << ", wheel_right=" << use_wheel_right);
    DBG_LOG(1, "CFG", "imu file='" << imupath << "', columns=" << imudatalen << ", rate_hz=" << imudatarate);
    DBG_LOG(1, "CFG", "gnss file='" << gnsspath << "', output='" << outputpath << "'");
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
        // Continue; subsequent file open checks will handle failure
    }
    if (imudatarate == 0 && config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();

    // consider the Earth's rotation
    bool isearth = yamlToBool(config["isearth"], true);

    GnssFileLoader gnssfile(gnsspath);
    bool imu_is_wheel = (use_wheel_left || use_wheel_right);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate, imu_is_wheel);
    // Optional additional IMU streams for multi-chain running
    std::unique_ptr<ImuFileLoader> imufile_left, imufile_right, imufile_main;
    if (imu_node_left) {
        std::string path = imu_node_left["file"] ? imu_node_left["file"].as<std::string>() : "";
        int cols = imu_node_left["columns"] ? imu_node_left["columns"].as<int>() : imudatalen;
        int rate = imu_node_left["rate_hz"] ? imu_node_left["rate_hz"].as<int>() : imudatarate;
        if (!path.empty()) imufile_left = std::make_unique<ImuFileLoader>(path, cols, rate, true);
    }
    if (imu_node_right) {
        std::string path = imu_node_right["file"] ? imu_node_right["file"].as<std::string>() : "";
        int cols = imu_node_right["columns"] ? imu_node_right["columns"].as<int>() : imudatalen;
        int rate = imu_node_right["rate_hz"] ? imu_node_right["rate_hz"].as<int>() : imudatarate;
        if (!path.empty()) imufile_right = std::make_unique<ImuFileLoader>(path, cols, rate, true);
    }
    if (imu_node_main) {
        std::string path = imu_node_main["file"] ? imu_node_main["file"].as<std::string>() : "";
        int cols = imu_node_main["columns"] ? imu_node_main["columns"].as<int>() : imudatalen;
        int rate = imu_node_main["rate_hz"] ? imu_node_main["rate_hz"].as<int>() : imudatarate;
        if (!path.empty()) imufile_main = std::make_unique<ImuFileLoader>(path, cols, rate, false);
    }
    DBG_LOG(1, "IO", "imu_is_wheel=" << imu_is_wheel);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    // 额外输出（多链同时保存）
    FileSaver navfile_left, errfile_left, navfile_right, errfile_right;
    bool left_out_ok = false, right_out_ok = false;
    if (save_multi_imu) {
        if (use_wheel_left) {
            std::string navL = outputpath + "/OB_GINS_TXT_left.nav";
            std::string errL = outputpath + "/OB_GINS_IMU_ERR_left.bin";
            left_out_ok = navfile_left.open(navL, 11, FileSaver::TEXT) && errfile_left.open(errL, 7, FileSaver::BINARY);
            if (!left_out_ok) {
                LOG(WARNING) << "[IO] 打开左轮输出失败: nav='" << navL << "' err='" << errL << "'";
            } else {
                LOG(INFO) << "[IO] 左轮输出文件已打开: nav='" << navL << "' err='" << errL << "'";
            }
        }
        if (use_wheel_right) {
            std::string navR = outputpath + "/OB_GINS_TXT_right.nav";
            std::string errR = outputpath + "/OB_GINS_IMU_ERR_right.bin";
            right_out_ok = navfile_right.open(navR, 11, FileSaver::TEXT) && errfile_right.open(errR, 7, FileSaver::BINARY);
            if (!right_out_ok) {
                LOG(WARNING) << "[IO] 打开右轮输出失败: nav='" << navR << "' err='" << errR << "'";
            } else {
                LOG(INFO) << "[IO] 右轮输出文件已打开: nav='" << navR << "' err='" << errR << "'";
            }
        }
    }
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;
    // Override bodyangle per-IMU if provided under the selected imu_node
    if (imu_node && imu_node["bodyangle"]) {
        vec = imu_node["bodyangle"].as<std::vector<double>>();
        Vector3d imu_bodyangle(vec.data());
        bodyangle = imu_bodyangle * D2R;
    }

    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();
    if (imu_node && imu_node["imunoise"]) {
        auto ino = imu_node["imunoise"];
        parameters->gyr_arw      = (ino["arw"].as<double>()) * D2R / 60.0;     // deg/sqrt(hr) -> rad/s^0.5
        parameters->acc_vrw      = (ino["vrw"].as<double>()) / 60.0;           // m/s/sqrt(hr) -> m/s^1.5
        parameters->gyr_bias_std = (ino["gbstd"].as<double>()) * D2R / 3600.0; // deg/hr -> rad/s
        parameters->acc_bias_std = (ino["abstd"].as<double>()) * 1.0e-5;       // mGal -> m/s^2
        parameters->corr_time    = (ino["corrtime"].as<double>()) * 3600.0;    // hr -> s
        if (ino["gsstd"]) parameters->gyr_scale_std = ino["gsstd"].as<double>() * 1e-6; // ppm -> ratio
        if (ino["asstd"]) parameters->acc_scale_std = ino["asstd"].as<double>() * 1e-6; // ppm -> ratio
    } else {
        parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
        parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
        parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
        parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
        parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;
    }

    bool isuseodo       = yamlToBool(config["odometer"]["isuseodo"], false);
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;
    DBG_LOG(2, "CFG",
            "imu noise: arw=" << parameters->gyr_arw << ", vrw=" << parameters->acc_vrw
            << ", gbstd=" << parameters->gyr_bias_std << ", abstd=" << parameters->acc_bias_std
            << ", corr_time=" << parameters->corr_time);
    DBG_LOG(2, "CFG",
            "odo: use=" << isuseodo << ", std=" << parameters->odo_std.transpose()
            << ", srw=" << parameters->odo_srw);

    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // Multi-chain mode if main+at least one wheel are enabled
    bool multi_chain = (use_main && (use_wheel_left || use_wheel_right) && imufile_main);
    if (multi_chain) {
        // Align each IMU stream to starttime
        IMU imu_cur_main{}, imu_pre_main{};
        IMU imu_cur_left{}, imu_pre_left{};
        IMU imu_cur_right{}, imu_pre_right{};
        do { imu_pre_main = imu_cur_main; imu_cur_main = imufile_main->next(); } while (imu_cur_main.time < starttime);
        if (use_wheel_left && imufile_left)
            do { imu_pre_left = imu_cur_left; imu_cur_left = imufile_left->next(); } while (imu_cur_left.time < starttime);
        if (use_wheel_right && imufile_right)
            do { imu_pre_right = imu_cur_right; imu_cur_right = imufile_right->next(); } while (imu_cur_right.time < starttime);
        DBG_LOG(2, "IMU", "aligned main: t_pre=" << imu_pre_main.time << ", t_cur=" << imu_cur_main.time);
        if (use_wheel_left && imufile_left)
            DBG_LOG(2, "IMU", "aligned left: t_pre=" << imu_pre_left.time << ", t_cur=" << imu_cur_left.time);
        if (use_wheel_right && imufile_right)
            DBG_LOG(2, "IMU", "aligned right: t_pre=" << imu_pre_right.time << ", t_cur=" << imu_cur_right.time);

        GNSS gnss;
        do { gnss = gnssfile.next(); } while (gnss.time < starttime);

        Vector3d station_origin = gnss.blh;
        parameters->gravity     = Earth::gravity(gnss.blh);
        gnss.blh                = Earth::global2local(station_origin, gnss.blh);
        parameters->station     = station_origin;

        std::vector<IntegrationState>       statelist_main(windows + 1);
        std::vector<IntegrationStateData>   statedatalist_main(windows + 1);
        std::deque<std::shared_ptr<Adapter::UnifiedPreintegrator>> preint_main;
        std::vector<WheelIntegrationState>     statelist_left(windows + 1), statelist_right(windows + 1);
        std::vector<WheelIntegrationStateData> statedatalist_left(windows + 1), statedatalist_right(windows + 1);
        std::deque<std::shared_ptr<Adapter::UnifiedPreintegrator>> preint_left, preint_right;
        std::deque<GNSS> gnsslist;
        std::deque<double> timelist;

        Adapter::UnifiedPreintegrator::Options preintegration_options = Adapter::GetOptions(isuseodo, isearth);

        IntegrationState state_curr_main = {
            .time = round(gnss.time),
            .p    = gnss.blh - Rotation::euler2quaternion(initatt_main) * antlever,
            .q    = Rotation::euler2quaternion(initatt_main),
            .v    = initvel,
            .bg   = initbg,
            .ba   = initba,
            .sodo = 0.0,
            .abv  = {bodyangle[1], bodyangle[2]},
        };
        std::cout << "Initilization at " << gnss.time << " s " << std::endl;
        DBG_LOG(1, "INIT", "p0=" << state_curr_main.p.transpose() << ", v0=" << state_curr_main.v.transpose()
                << ", att0(deg)=" << (Rotation::quaternion2euler(state_curr_main.q) * R2D).transpose());

        statelist_main[0]     = state_curr_main;
        statedatalist_main[0] = Adapter::StateToData(state_curr_main, preintegration_options);
        if (use_wheel_left) {
            WheelIntegrationState ws{};
            ws.time = state_curr_main.time; ws.p = state_curr_main.p; ws.q = state_curr_main.q; ws.v = state_curr_main.v;
            ws.bg = state_curr_main.bg; ws.ba = state_curr_main.ba; ws.s = state_curr_main.s; ws.sodo = state_curr_main.sodo;
            ws.abv = state_curr_main.abv; ws.sg = state_curr_main.sg; ws.sa = state_curr_main.sa;
            // 左链初始姿态覆盖
            ws.q = Rotation::euler2quaternion(initatt_left);
            statelist_left[0]      = ws;
            statedatalist_left[0]  = Adapter::StateToDataWheel(ws, preintegration_options);
        }
        if (use_wheel_right) {
            WheelIntegrationState ws{};
            ws.time = state_curr_main.time; ws.p = state_curr_main.p; ws.q = state_curr_main.q; ws.v = state_curr_main.v;
            ws.bg = state_curr_main.bg; ws.ba = state_curr_main.ba; ws.s = state_curr_main.s; ws.sodo = state_curr_main.sodo;
            ws.abv = state_curr_main.abv; ws.sg = state_curr_main.sg; ws.sa = state_curr_main.sa;
            // 右链初始姿态覆盖
            ws.q = Rotation::euler2quaternion(initatt_right);
            statelist_right[0]     = ws;
            statedatalist_right[0] = Adapter::StateToDataWheel(ws, preintegration_options);
        }
        gnsslist.push_back(gnss);
        double sow = round(gnss.time);
        timelist.push_back(sow);

        // Initial preintegrations per chain（左右链使用各自初始姿态）
        preint_main.emplace_back(Adapter::UnifiedPreintegrator::Create(
            parameters, imu_pre_main, state_curr_main, preintegration_options, false));
        if (use_wheel_left) {
            IntegrationState init_left = state_curr_main; init_left.q = Rotation::euler2quaternion(initatt_left);
            preint_left.emplace_back(Adapter::UnifiedPreintegrator::Create(
                parameters, imu_pre_left, init_left, preintegration_options, true));
        }
        if (use_wheel_right) {
            IntegrationState init_right = state_curr_main; init_right.q = Rotation::euler2quaternion(initatt_right);
            preint_right.emplace_back(Adapter::UnifiedPreintegrator::Create(
                parameters, imu_pre_right, init_right, preintegration_options, true));
        }

        // prime next gnss
        gnss                = gnssfile.next();
        parameters->gravity = Earth::gravity(gnss.blh);
        gnss.blh            = Earth::global2local(station_origin, gnss.blh);

        std::shared_ptr<MarginalizationInfo> last_marginalization_info;
        std::vector<double *> last_marginalization_parameter_blocks;

        sow += integration_length;

        while (true) {
            if ((imu_cur_main.time > endtime) || imufile_main->isEof()) {
                break;
            }

            // feed master (main) stream
            preint_main.back()->addNewImu(imu_cur_main);
            imu_pre_main = imu_cur_main;
            imu_cur_main = imufile_main->next();

            if (imu_cur_main.time > sow) {
                // GNSS alignment
                if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
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
                    parameters->gravity = Earth::gravity(gnss.blh);
                    gnss.blh            = Earth::global2local(station_origin, gnss.blh);
                    if (gnssfile.isEof()) { gnss.time = 0; }
                }

                // boundary interpolation for main
                int isneed = isNeedInterpolation(imu_pre_main, imu_cur_main, sow);
                if (isneed == 1) {
                    preint_main.back()->addNewImu(imu_cur_main);
                    imu_pre_main = imu_cur_main; imu_cur_main = imufile_main->next();
                } else if (isneed == 2) {
                    imuInterpolation(imu_cur_main, imu_pre_main, imu_cur_main, sow);
                    preint_main.back()->addNewImu(imu_pre_main);
                }

                // catch up wheel-left/right to boundary
                if (use_wheel_left && imufile_left) {
                    while (imu_cur_left.time <= sow) { preint_left.back()->addNewImu(imu_cur_left); imu_pre_left = imu_cur_left; imu_cur_left = imufile_left->next(); }
                    int needL = isNeedInterpolation(imu_pre_left, imu_cur_left, sow);
                    if (needL == 1) { preint_left.back()->addNewImu(imu_cur_left); imu_pre_left = imu_cur_left; imu_cur_left = imufile_left->next(); }
                    else if (needL == 2) { imuInterpolation(imu_cur_left, imu_pre_left, imu_cur_left, sow); preint_left.back()->addNewImu(imu_pre_left); }
                }
                if (use_wheel_right && imufile_right) {
                    while (imu_cur_right.time <= sow) { preint_right.back()->addNewImu(imu_cur_right); imu_pre_right = imu_cur_right; imu_cur_right = imufile_right->next(); }
                    int needR = isNeedInterpolation(imu_pre_right, imu_cur_right, sow);
                    if (needR == 1) { preint_right.back()->addNewImu(imu_cur_right); imu_pre_right = imu_cur_right; imu_cur_right = imufile_right->next(); }
                    else if (needR == 2) { imuInterpolation(imu_cur_right, imu_pre_right, imu_cur_right, sow); preint_right.back()->addNewImu(imu_pre_right); }
                }

                // push time node
                timelist.push_back(sow);
                // 杈撳嚭杩愯杩涘害锛堝閾撅級
                {
                    static int lastpercent = -1;
                    int percent = int(((timelist.back() - starttime) * 100.0) / (endtime - starttime));
                    if (percent < 0) percent = 0; if (percent > 100) percent = 100;
                    if (percent != lastpercent) {
                        lastpercent = percent;
                        std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                        flush(std::cout);
                    }
                }
                sow += integration_length;

                // 在进入优化/发布阶段前，打印 PI 入口统计（使用实际参与预积分的输入序列）
                if (Debug::on(1)) {
                    auto mean_acc_norm = [](const std::vector<IMU> &buf) -> double {
                        double asum = 0.0; size_t cnt = 0;
                        for (const auto &m : buf) { if (m.dt > 0) { asum += (m.dvel / m.dt).norm(); cnt++; } }
                        return cnt ? (asum / static_cast<double>(cnt)) : 0.0;
                    };
                    if (!preint_main.empty()) {
                        const auto &used = preint_main.back()->inputView();
                        if (!used.empty()) {
                            LOG(INFO) << "[PI-IN] main N=" << used.size()
                                      << " t:[" << used.front().time << "," << used.back().time << "]"
                                      << " ā=" << mean_acc_norm(used);
                        }
                    }
                    if (use_wheel_left && !preint_left.empty()) {
                        const auto &used = preint_left.back()->inputView();
                        if (!used.empty()) {
                            LOG(INFO) << "[PI-IN] left N=" << used.size()
                                      << " t:[" << used.front().time << "," << used.back().time << "]"
                                      << " ā=" << mean_acc_norm(used);
                        }
                    }
                    if (use_wheel_right && !preint_right.empty()) {
                        const auto &used = preint_right.back()->inputView();
                        if (!used.empty()) {
                            LOG(INFO) << "[PI-IN] right N=" << used.size()
                                      << " t:[" << used.front().time << "," << used.back().time << "]"
                                      << " ā=" << mean_acc_norm(used);
                        }
                    }
                }

                // fill end states
                auto st_main = preint_main.back()->currentStateMain();
                statelist_main[preint_main.size()]     = st_main;
                statedatalist_main[preint_main.size()] = Adapter::StateToData(st_main, preintegration_options);
                if (use_wheel_left) {
                    auto st = preint_left.back()->currentStateWheel();
                    statelist_left[preint_left.size()]     = st;
                    statedatalist_left[preint_left.size()] = Adapter::StateToDataWheel(st, preintegration_options);
                }
                if (use_wheel_right) {
                    auto st = preint_right.back()->currentStateWheel();
                    statelist_right[preint_right.size()]     = st;
                    statedatalist_right[preint_right.size()] = Adapter::StateToDataWheel(st, preintegration_options);
                }

                // Minimal self-checks at keyframe t_k: per-chain IMU slice stats and addresses
                if (Debug::on(1)) {
                    auto print_slice = [](const char *name, const std::vector<IMU> &buf) {
                        size_t N = buf.size();
                        double t0 = N ? buf.front().time : 0.0;
                        double t1 = N ? buf.back().time  : 0.0;
                        // mean |a| approximated by |dvel/dt|
                        double asum = 0.0; size_t cnt = 0;
                        for (const auto &m : buf) {
                            if (m.dt > 0) { asum += (m.dvel / m.dt).norm(); cnt++; }
                        }
                        double a_bar = cnt ? (asum / static_cast<double>(cnt)) : 0.0;
                        DBG_LOG(1, "CHAIN", name << " slice: N=" << N << ", [" << t0 << "," << t1 << "] a_bar=" << a_bar);
                    };
                    // main chain
                    if (!preint_main.empty() && preint_main.back()->rawMain())
                        print_slice("main", preint_main.back()->rawMain()->imuBuffer());
                    // left wheel chain
                    if (use_wheel_left && !preint_left.empty() && preint_left.back()->rawWheel())
                        print_slice("left", preint_left.back()->rawWheel()->imuBuffer());
                    // right wheel chain
                    if (use_wheel_right && !preint_right.empty() && preint_right.back()->rawWheel())
                        print_slice("right", preint_right.back()->rawWheel()->imuBuffer());

                    // Print addresses of State objects and per-chain preintegrator instances
                    DBG_LOG(1, "CHAIN",
                            "main state@" << &statelist_main[preint_main.size()] << " est@" << preint_main.back().get());
                    if (use_wheel_left) {
                        DBG_LOG(1, "CHAIN",
                                "left state@" << &statelist_left[preint_left.size()] << " est@" << preint_left.back().get());
                    }
                    if (use_wheel_right) {
                        DBG_LOG(1, "CHAIN",
                                "right state@" << &statelist_right[preint_right.size()] << " est@" << preint_right.back().get());
                    }
                }

                // build and solve ceres problem
                {
                    ceres::Problem::Options problem_options; problem_options.enable_fast_removal = true;
                    ceres::Problem problem(problem_options);
                    ceres::Solver solver; ceres::Solver::Summary summary; ceres::Solver::Options options;
                    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
                    options.num_threads                = 4;

                    // add parameter blocks for all chains
                    for (size_t k = 0; k <= preint_main.size(); k++) {
                        ceres::Manifold *manifold = new PoseManifold();
                        problem.AddParameterBlock(statedatalist_main[k].pose, Adapter::NumPoseParameter(), manifold);
                        problem.AddParameterBlock(statedatalist_main[k].mix, Adapter::NumMixParameter(preintegration_options));
                    }
                    if (use_wheel_left) {
                        for (size_t k = 0; k <= preint_left.size(); k++) {
                            ceres::Manifold *manifold = new PoseManifold();
                            problem.AddParameterBlock(statedatalist_left[k].pose, Adapter::NumPoseParameterWheel(), manifold);
                            problem.AddParameterBlock(statedatalist_left[k].mix, Adapter::NumMixParameterWheel(preintegration_options));
                        }
                    }
                    if (use_wheel_right) {
                        for (size_t k = 0; k <= preint_right.size(); k++) {
                            ceres::Manifold *manifold = new PoseManifold();
                            problem.AddParameterBlock(statedatalist_right[k].pose, Adapter::NumPoseParameterWheel(), manifold);
                            problem.AddParameterBlock(statedatalist_right[k].mix, Adapter::NumMixParameterWheel(preintegration_options));
                        }
                    }

                    // GNSS 鍚屾鍔犲叆涓?杞摼
                    int index = 0; ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
                    std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id;
                    for (const auto &g : gnsslist) {
                        for (size_t i = index; i <= preint_main.size(); ++i) {
                            if (fabs(g.time - timelist[i]) < MINIMUM_INTERVAL) {
                                // 涓婚摼
                                {
                                    auto factor_m = new GnssFactor(g, antlever);
                                    auto id = problem.AddResidualBlock(factor_m, loss_function, statedatalist_main[i].pose);
                                    gnss_residualblock_id.push_back(std::make_pair(g.time, id));
                                }
                                // 宸?鍙宠疆閾撅紙濡傚惎鐢級
                                if (use_wheel_left) {
                                    auto factor_l = new GnssFactor(g, antlever);
                                    problem.AddResidualBlock(factor_l, loss_function, statedatalist_left[i].pose);
                                }
                                if (use_wheel_right) {
                                    auto factor_r = new GnssFactor(g, antlever);
                                    problem.AddResidualBlock(factor_r, loss_function, statedatalist_right[i].pose);
                                }
                                index++;
                                break;
                            }
                        }
                    }

                    // preintegration factors
                    for (size_t k = 0; k < preint_main.size(); k++) {
                        auto factor = Adapter::MakePreintFactor(preint_main[k]);
                        problem.AddResidualBlock(factor, nullptr, statedatalist_main[k].pose, statedatalist_main[k].mix,
                                                 statedatalist_main[k + 1].pose, statedatalist_main[k + 1].mix);
                    }
                    if (use_wheel_left) {
                        for (size_t k = 0; k < preint_left.size(); k++) {
                            auto factor = Adapter::MakePreintFactor(preint_left[k]);
                            problem.AddResidualBlock(factor, nullptr, statedatalist_left[k].pose, statedatalist_left[k].mix,
                                                     statedatalist_left[k + 1].pose, statedatalist_left[k + 1].mix);
                        }
                    }
                    if (use_wheel_right) {
                        for (size_t k = 0; k < preint_right.size(); k++) {
                            auto factor = Adapter::MakePreintFactor(preint_right[k]);
                            problem.AddResidualBlock(factor, nullptr, statedatalist_right[k].pose, statedatalist_right[k].mix,
                                                     statedatalist_right[k + 1].pose, statedatalist_right[k + 1].mix);
                        }
                    }
                    {
                        // bias constraint per chain at window tail
                        auto factor = Adapter::MakeImuErrorFactor(preint_main.back());
                        problem.AddResidualBlock(factor, nullptr, statedatalist_main[preint_main.size()].mix);
                        if (use_wheel_left) {
                            auto fL = Adapter::MakeImuErrorFactor(preint_left.back());
                            problem.AddResidualBlock(fL, nullptr, statedatalist_left[preint_left.size()].mix);
                        }
                        if (use_wheel_right) {
                            auto fR = Adapter::MakeImuErrorFactor(preint_right.back());
                            problem.AddResidualBlock(fR, nullptr, statedatalist_right[preint_right.size()].mix);
                        }
                    }

                    if (last_marginalization_info && last_marginalization_info->isValid()) {
                        auto factor = new MarginalizationFactor(last_marginalization_info);
                        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                    }

                    options.max_num_iterations = num_iterations / 4; solver.Solve(options, &problem, &summary);

                    // outlier culling on main GNSS
                    if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                        double chi2_threshold = 7.815;
                        std::unordered_set<double> gnss_outlier;
                        size_t K = std::min(gnsslist.size(), gnss_residualblock_id.size());
                        for (size_t k = 0; k < K; k++) {
                            auto time = gnss_residualblock_id[k].first; auto id = gnss_residualblock_id[k].second;
                            double cost; problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
                            double chi2 = cost * 2;
                            if (chi2 > chi2_threshold) { gnss_outlier.insert(time); double scale = sqrt(chi2 / chi2_threshold); gnsslist[k].std *= scale; }
                        }
                        for (const auto &block : gnss_residualblock_id) { problem.RemoveResidualBlock(block.second); }
                        index = 0; for (auto &g2 : gnsslist) {
                            for (size_t i = index; i <= preint_main.size(); ++i) {
                                if (fabs(g2.time - timelist[i]) < MINIMUM_INTERVAL) {
                                    auto fM = new GnssFactor(g2, antlever);
                                    problem.AddResidualBlock(fM, nullptr, statedatalist_main[i].pose);
                                    if (use_wheel_left) { auto fL = new GnssFactor(g2, antlever); problem.AddResidualBlock(fL, nullptr, statedatalist_left[i].pose); }
                                    if (use_wheel_right) { auto fR = new GnssFactor(g2, antlever); problem.AddResidualBlock(fR, nullptr, statedatalist_right[i].pose); }
                                    index++; break;
                                }
                            }
                        }
                    }

                    // marginalization when window full
                    if (preint_main.size() == static_cast<size_t>(windows)) {
                        std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                        if (last_marginalization_info && last_marginalization_info->isValid()) {
                            std::vector<int> marginilized_index;
                            for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                                if (last_marginalization_parameter_blocks[k] == statedatalist_main[0].pose ||
                                    last_marginalization_parameter_blocks[k] == statedatalist_main[0].mix) {
                                    marginilized_index.push_back(static_cast<int>(k));
                                }
                                if (use_wheel_left &&
                                    (last_marginalization_parameter_blocks[k] == statedatalist_left[0].pose ||
                                     last_marginalization_parameter_blocks[k] == statedatalist_left[0].mix)) {
                                    marginilized_index.push_back(static_cast<int>(k));
                                }
                                if (use_wheel_right &&
                                    (last_marginalization_parameter_blocks[k] == statedatalist_right[0].pose ||
                                     last_marginalization_parameter_blocks[k] == statedatalist_right[0].mix)) {
                                    marginilized_index.push_back(static_cast<int>(k));
                                }
                            }
                            auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                            auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                            marginalization_info->addResidualBlockInfo(residual);
                        }

                        // add first segment factors for all chains
                        {
                            auto factor   = std::shared_ptr<ceres::CostFunction>(Adapter::MakePreintFactor(preint_main[0]));
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr,
                                std::vector<double *>{statedatalist_main[0].pose, statedatalist_main[0].mix,
                                                      statedatalist_main[1].pose, statedatalist_main[1].mix},
                                std::vector<int>{0, 1});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                        if (use_wheel_left) {
                            auto factor   = std::shared_ptr<ceres::CostFunction>(Adapter::MakePreintFactor(preint_left[0]));
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr,
                                std::vector<double *>{statedatalist_left[0].pose, statedatalist_left[0].mix,
                                                      statedatalist_left[1].pose, statedatalist_left[1].mix},
                                std::vector<int>{0, 1});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                        if (use_wheel_right) {
                            auto factor   = std::shared_ptr<ceres::CostFunction>(Adapter::MakePreintFactor(preint_right[0]));
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr,
                                std::vector<double *>{statedatalist_right[0].pose, statedatalist_right[0].mix,
                                                      statedatalist_right[1].pose, statedatalist_right[1].mix},
                                std::vector<int>{0, 1});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, std::vector<double *>{statedatalist_main[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }

                        marginalization_info->marginalization();

                        std::unordered_map<std::uintptr_t, double *> address;
                        for (size_t k = 1; k <= preint_main.size(); k++) {
                            address[reinterpret_cast<std::uintptr_t>(statedatalist_main[k].pose)] = statedatalist_main[k - 1].pose;
                            address[reinterpret_cast<std::uintptr_t>(statedatalist_main[k].mix)]  = statedatalist_main[k - 1].mix;
                        }
                        if (use_wheel_left) {
                            for (size_t k = 1; k <= preint_left.size(); k++) {
                                address[reinterpret_cast<std::uintptr_t>(statedatalist_left[k].pose)] = statedatalist_left[k - 1].pose;
                                address[reinterpret_cast<std::uintptr_t>(statedatalist_left[k].mix)]  = statedatalist_left[k - 1].mix;
                            }
                        }
                        if (use_wheel_right) {
                            for (size_t k = 1; k <= preint_right.size(); k++) {
                                address[reinterpret_cast<std::uintptr_t>(statedatalist_right[k].pose)] = statedatalist_right[k - 1].pose;
                                address[reinterpret_cast<std::uintptr_t>(statedatalist_right[k].mix)]  = statedatalist_right[k - 1].mix;
                            }
                        }
                        last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                        last_marginalization_info             = std::move(marginalization_info);

                        // pop oldest segment across chains
                        if (lround(timelist[0]) == lround(gnsslist[0].time)) { gnsslist.pop_front(); }
                        timelist.pop_front();
                        preint_main.pop_front();
                        if (use_wheel_left)  preint_left.pop_front();
                        if (use_wheel_right) preint_right.pop_front();

                        for (int k = 0; k < windows; k++) {
                            statedatalist_main[k] = statedatalist_main[k + 1];
                            statelist_main[k]     = Adapter::StateFromData(statedatalist_main[k], preintegration_options);
                            if (use_wheel_left) {
                                statedatalist_left[k] = statedatalist_left[k + 1];
                                statelist_left[k]     = Adapter::StateFromDataWheel(statedatalist_left[k], preintegration_options);
                            }
                            if (use_wheel_right) {
                                statedatalist_right[k] = statedatalist_right[k + 1];
                                statelist_right[k]     = Adapter::StateFromDataWheel(statedatalist_right[k], preintegration_options);
                            }
                        }
                    }
                }

                // write results at boundary（主链 + 可选左右轮链）。发布前打印将要发布的拷贝
                {
                    IntegrationState out = statelist_main[preint_main.size()];
                    LOG(INFO) << "[PUB-IN] main p=" << out.p.transpose() << " q=" << out.q.coeffs().transpose();
                    writeNavResult(*timelist.rbegin(), station_origin, out, navfile, errfile);
                }
                if (save_multi_imu && use_wheel_left && navfile_left.isOpen() && errfile_left.isOpen()) {
                    auto outL = statelist_left[preint_left.size()];
                    LOG(INFO) << "[PUB-IN] left p=" << outL.p.transpose() << " q=" << outL.q.coeffs().transpose();
                    writeNavResultWheel(*timelist.rbegin(), station_origin, outL, navfile_left, errfile_left);
                }
                if (save_multi_imu && use_wheel_right && navfile_right.isOpen() && errfile_right.isOpen()) {
                    auto outR = statelist_right[preint_right.size()];
                    LOG(INFO) << "[PUB-IN] right p=" << outR.p.transpose() << " q=" << outR.q.coeffs().transpose();
                    writeNavResultWheel(*timelist.rbegin(), station_origin, outR, navfile_right, errfile_right);
                }

                // start next segment preintegrations（左右链从各自末态起步）
                auto toIntegration = [](const WheelIntegrationState &ws) -> IntegrationState {
                    IntegrationState s{}; s.time = ws.time; s.p = ws.p; s.q = ws.q; s.v = ws.v;
                    s.bg = ws.bg; s.ba = ws.ba; s.s = ws.s; s.sodo = ws.sodo; s.abv = ws.abv; s.sg = ws.sg; s.sa = ws.sa; return s;
                };
                preint_main.emplace_back(Adapter::UnifiedPreintegrator::Create(
                    parameters, imu_pre_main, statelist_main[preint_main.size()], preintegration_options, false));
                if (use_wheel_left) {
                    IntegrationState left_state = toIntegration(statelist_left[preint_left.size()]);
                    preint_left.emplace_back(Adapter::UnifiedPreintegrator::Create(
                        parameters, imu_pre_left, left_state, preintegration_options, true));
                }
                if (use_wheel_right) {
                    IntegrationState right_state = toIntegration(statelist_right[preint_right.size()]);
                    preint_right.emplace_back(Adapter::UnifiedPreintegrator::Create(
                        parameters, imu_pre_right, right_state, preintegration_options, true));
                }
            } else {
                // streaming output between keyframes（主链 + 可选左右轮链）
                auto integration = *preint_main.rbegin();
                double t_out = integration->endTime();
                // 发布前打印将要发布的拷贝
                {
                    IntegrationState out = integration->currentStateMain();
                    LOG(INFO) << "[PUB-IN] main p=" << out.p.transpose() << " q=" << out.q.coeffs().transpose();
                    writeNavResult(t_out, station_origin, out, navfile, errfile);
                }
                if (save_multi_imu && use_wheel_left && navfile_left.isOpen() && errfile_left.isOpen()) {
                    auto outL = preint_left.back()->currentStateWheel();
                    LOG(INFO) << "[PUB-IN] left p=" << outL.p.transpose() << " q=" << outL.q.coeffs().transpose();
                    writeNavResultWheel(t_out, station_origin, outL, navfile_left, errfile_left);
                }
                if (save_multi_imu && use_wheel_right && navfile_right.isOpen() && errfile_right.isOpen()) {
                    auto outR = preint_right.back()->currentStateWheel();
                    LOG(INFO) << "[PUB-IN] right p=" << outR.p.transpose() << " q=" << outR.q.coeffs().transpose();
                    writeNavResultWheel(t_out, station_origin, outR, navfile_right, errfile_right);
                }
            }
        }

        navfile.close(); errfile.close();
        if (navfile_left.isOpen()) navfile_left.close();
        if (errfile_left.isOpen()) errfile_left.close();
        if (navfile_right.isOpen()) navfile_right.close();
        if (errfile_right.isOpen()) errfile_right.close();
        if (imufile_main) imufile_main->close(); if (imufile_left) imufile_left->close(); if (imufile_right) imufile_right->close();
        gnssfile.close();

        auto te = std::chrono::steady_clock::now();
        std::cout << std::endl << std::endl << "Cost " << std::chrono::duration<double>(te - ts).count() << " s in total" << std::endl;
        return 0;
    }

    // data alignment (single-chain legacy path)
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur;
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);
    DBG_LOG(2, "IMU", "aligned to starttime: t_pre=" << imu_pre.time << ", t_cur=" << imu_cur.time
            << ", dt=" << imu_cur.dt << ", odovel=" << imu_cur.odovel);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    Vector3d station_origin = gnss.blh;
    parameters->gravity     = Earth::gravity(gnss.blh);
    gnss.blh                = Earth::global2local(station_origin, gnss.blh);
    // Fill station origin for Earth-rotation related models
    parameters->station     = station_origin;
    DBG_LOG(2, "GNSS", "init gnss t=" << gnss.time << ", blh(local)=" << gnss.blh.transpose()
            << ", std=" << gnss.std.transpose());


    // Wheel preintegration wiring was removed per request; proceed with main IMU chain.

    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::deque<std::shared_ptr<Adapter::UnifiedPreintegrator>> preintegrationlist;
    std::deque<GNSS> gnsslist;
    std::deque<double> timelist;

    Adapter::UnifiedPreintegrator::Options preintegration_options = Adapter::GetOptions(isuseodo, isearth);

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
    DBG_LOG(1, "INIT", "p0=" << state_curr.p.transpose() << ", v0=" << state_curr.v.transpose()
            << ", att0(deg)=" << (Rotation::quaternion2euler(state_curr.q) * R2D).transpose());

    statelist[0]     = state_curr;
    statedatalist[0] = Adapter::StateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // Initial preintegration
    preintegrationlist.emplace_back(
        Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr, preintegration_options,
                                              imu_pre.is_wheel));
    std::shared_ptr<Adapter::UnifiedPreintegrator> shadow_preintegration;
    if (dbg_dual_chain && imu_pre.is_wheel) {
        shadow_preintegration = Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr,
                                                                      preintegration_options, false);
        DBG_LOG(1, "DBG", "dual_chain enabled: comparing WHEEL vs MAIN preintegration");
    }

    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh            = Earth::global2local(station_origin, gnss.blh);

    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    sow += integration_length;

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
            break;
        }

        // Add new imu data to preintegration 
        preintegrationlist.back()->addNewImu(imu_cur);
        if (shadow_preintegration) shadow_preintegration->addNewImu(imu_cur);

        imu_pre = imu_cur;
        imu_cur = imufile.next();

        if (imu_cur.time > sow) {
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
                gnsslist.push_back(gnss);

                gnss = gnssfile.next();
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold)) {
                    gnss = gnssfile.next();
                }

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

            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) {
            } else if (isneed == 1) {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = imufile.next();
            } else if (isneed == 2) {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
                if (shadow_preintegration) shadow_preintegration->addNewImu(imu_pre);
            }

            // next time node
            timelist.push_back(sow);
            sow += integration_length;

            state_curr                               = preintegrationlist.back()->currentStateMain();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Adapter::StateToData(state_curr, preintegration_options);
            if (shadow_preintegration) {
                auto state_shadow = shadow_preintegration->currentStateMain();
                auto euler_curr   = Rotation::quaternion2euler(state_curr.q) * R2D;
                auto euler_sh     = Rotation::quaternion2euler(state_shadow.q) * R2D;
                DBG_LOG(1, "DUAL",
                        "t=" << timelist.back() << ", pos(m): wheel=" << state_curr.p.transpose()
                        << ", main=" << state_shadow.p.transpose()
                        << "; att(deg): wheel=" << euler_curr.transpose() << ", main=" << euler_sh.transpose()
                        << "; vel(m/s): wheel=" << state_curr.v.transpose() << ", main=" << state_shadow.v.transpose());
                DBG_LOG(2, "DUAL",
                        "bias_g(dph): wheel=" << (state_curr.bg * R2D * 3600).transpose()
                        << ", main=" << (state_shadow.bg * R2D * 3600).transpose()
                        << "; bias_a(mGal): wheel=" << (state_curr.ba * 1e5).transpose()
                        << ", main=" << (state_shadow.ba * 1e5).transpose());
            }

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

                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    ceres::Manifold *manifold = new PoseManifold();
                    problem.AddParameterBlock(statedatalist[k].pose, Adapter::NumPoseParameter(), manifold);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Adapter::NumMixParameter(preintegration_options));
                }

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

                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = Adapter::MakePreintFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                {
                    // add IMU bias-constraint factors
                    auto factor = Adapter::MakeImuErrorFactor(preintegrationlist.back());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                options.max_num_iterations = num_iterations / 4;
                solver.Solve(options, &problem, &summary);
                DBG_LOG(2, "CERES", "first-pass: iter=" << summary.iterations.size()
                        << ", cost0=" << summary.initial_cost << ", cost1=" << summary.final_cost);

                // TODO: Just a example, you need remodify.
                // Do GNSS outlier culling using chi-square test
                if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                    // 3 degrees of freedom, 0.05
                    double chi2_threshold = 7.815;

                    // Find GNSS outliers in the window
                    std::unordered_set<double> gnss_outlier;
                    size_t K = std::min(gnsslist.size(), gnss_residualblock_id.size());
                    for (size_t k = 0; k < K; k++) {
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
                    if (Debug::on(2) && !gnss_outlier.empty()) {
                        std::ostringstream oss;
                        oss << "Reweight GNSS outlier at t=" << (sow - 1) << ":";
                        for (const auto &time : gnss_outlier) {
                            oss << " " << time;
                        }
                        Debug::print(2, "GNSS", oss.str());
                    }

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
                DBG_LOG(2, "CERES", "second-pass: iter=" << summary.iterations.size()
                        << ", cost0=" << summary.initial_cost << ", cost1=" << summary.final_cost);

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

                    {
                        auto factor   = std::shared_ptr<ceres::CostFunction>(Adapter::MakePreintFactor(preintegrationlist[0]));
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    marginalization_info->marginalization();

                    std::unordered_map<std::uintptr_t, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<std::uintptr_t>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<std::uintptr_t>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                }

                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time)) {
                        gnsslist.pop_front();
                    }
                    timelist.pop_front();
                    preintegrationlist.pop_front();

                    for (int k = 0; k < windows; k++) {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k]     = Adapter::StateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Adapter::StateFromData(statedatalist[windows], preintegration_options);
                    state_curr         = statelist[windows];
                }
            } else {
                state_curr =
                    Adapter::StateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            // write result
            writeNavResult(*timelist.rbegin(), station_origin, state_curr, navfile, errfile);

            preintegrationlist.emplace_back(
                Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr, preintegration_options,
                                                      imu_pre.is_wheel));
            if (shadow_preintegration) {
                shadow_preintegration = Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr,
                                                                             preintegration_options, false);
            }
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentStateMain(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    gnssfile.close();

    auto te = std::chrono::steady_clock::now();
    std::cout << std::endl
              << std::endl
              << "Cost "
              << std::chrono::duration<double>(te - ts).count()
              << " s in total" << std::endl;

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

void writeNavResultWheel(double time, const Vector3d &origin, const WheelIntegrationState &state, FileSaver &navfile,
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
    imu00.odovel = buff.odovel * (1 - scale);
    imu00.is_wheel = buff.is_wheel;

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
    imu11.is_wheel = buff.is_wheel;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        return 2;
    }

    return 0;
}








