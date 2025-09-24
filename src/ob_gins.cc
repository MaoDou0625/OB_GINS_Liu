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
// Wheel-IMU state for wheel-specific output
#include "src/wheel/integration_state_wheel.h"
// (Wheel preintegration headers were included temporarily for wiring; removed per request.)

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);
// Wheel-only writer to allow saving dual navigation results later
void writeNavResultWheel(double time, const Vector3d &origin, const WheelIntegrationState &state, FileSaver &navfile,
                         FileSaver &errfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";

    auto ts = absl::Now();

    // 璇诲彇閰嶇疆
    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // 鏃堕棿淇℃伅
    // processing time
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // 杩唬娆℃暟
    // number of iterations
    int num_iterations = config["num_iterations"].as<int>();

    // 杩涜GNSS绮楀樊鍓旈櫎
    // Do GNSS outlier culling
    bool is_outlier_culling = config["is_outlier_culling"].as<bool>();

    // 鍒濆鍖栦俊鎭?    // initialization
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

    // 鏁版嵁鏂囦欢
    // data file
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string imupath;
    int imudatalen  = 0;
    int imudatarate = 0;
    std::string outputpath = config["outputpath"].as<std::string>();

    // 鏂扮増澶欼MU閰嶇疆锛堜紭鍏堜簬鏃у瓧娈碉級
    bool use_main = false, use_wheel_left = false, use_wheel_right = false;
    if (config["run_mode"]) {
        auto rm = config["run_mode"];
        if (rm["imu_main_enable"])    use_main       = rm["imu_main_enable"].as<bool>();
        if (rm["wheel_left_enable"])  use_wheel_left = rm["wheel_left_enable"].as<bool>();
        if (rm["wheel_right_enable"]) use_wheel_right= rm["wheel_right_enable"].as<bool>();
    }

    // 璇诲彇鏂版 IMU 鍩烘湰閰嶇疆锛堣嫢涓嶅瓨鍦ㄥ垯鍥為€€鍒版棫瀛楁锛?
    YAML::Node imu_node;
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
        // 鍏煎鏃х増瀛楁
        // 閸忕厧顔愰弮褏澧楃€涙顔?
        if (config["imufile"])     imupath     = config["imufile"].as<std::string>();
        if (config["imudatalen"])  imudatalen  = config["imudatalen"].as<int>();
        if (config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();
    }
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
    // 鑻ユ湭鍦ㄦ柊閰嶇疆涓祴鍊硷紝纭繚浠嶆湁鏈夋晥榛樿鍊?    if (imudatalen == 0 && config["imudatalen"])  imudatalen  = config["imudatalen"].as<int>();
    if (imudatarate == 0 && config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();

    // 鏄惁鑰冭檻鍦扮悆鑷浆
    // consider the Earth's rotation
    bool isearth = config["isearth"].as<bool>();

    GnssFileLoader gnssfile(gnsspath);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // 瀹夎鍙傛暟
    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;

    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();
    //浠庢柊imunoise 璇诲彇锛堝瓨鍦ㄥ垯瑕嗙洊锛夛紝鍚﹀垯浣跨敤鏃у瓧imumodel
    if (imu_node && imu_node["imunoise"]) {
        auto ino = imu_node["imunoise"];
        parameters->gyr_arw      = (ino["arw"].as<double>()) * D2R / 60.0;     // deg/sqrt(hr) -> rad/s^0.5
        parameters->acc_vrw      = (ino["vrw"].as<double>()) / 60.0;           // m/s/sqrt(hr) -> m/s^1.5
        parameters->gyr_bias_std = (ino["gbstd"].as<double>()) * D2R / 3600.0; // deg/hr -> rad/s
        parameters->acc_bias_std = (ino["abstd"].as<double>()) * 1.0e-5;       // mGal -> m/s^2
        parameters->corr_time    = (ino["corrtime"].as<double>()) * 3600.0;    // hr -> s
        // 姣斾緥鍥犲瓙
        if (ino["gsstd"]) parameters->gyr_scale_std = ino["gsstd"].as<double>() * 1e-6; // ppm -> ratio
        if (ino["asstd"]) parameters->acc_scale_std = ino["asstd"].as<double>() * 1e-6; // ppm -> ratio
    } else {
        parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
        parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
        parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
        parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
        parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;
    }

    bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;

    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // 鏁版嵁瀵归綈
    // data alignment
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur;
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    // 鍒濆鍖栦綅缃紝璁＄畻閲嶅姏
    Vector3d station_origin = gnss.blh;
    parameters->gravity     = Earth::gravity(gnss.blh);
    gnss.blh                = Earth::global2local(station_origin, gnss.blh);

    // 绔欏績鍧愭爣绯诲師鐐?    parameters->station = station_origin;

    // Wheel preintegration wiring was removed per request; proceed with main IMU chain.

    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::deque<std::shared_ptr<PreintegrationBase>> preintegrationlist;
    std::deque<GNSS> gnsslist;
    std::deque<double> timelist;

    Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(isuseodo, isearth);

    // 鍒濆鐘舵€?    // initialization
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

    statelist[0]     = state_curr;
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // 鍒濆鍖栭绉垎
    // Initial preintegration
    preintegrationlist.emplace_back(
        Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));

    // 璇诲彇涓嬩竴鏉?GNSS
    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh            = Earth::global2local(station_origin, gnss.blh);

    // 边缘化信息
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

// 下一个时间节点
    sow += INTEGRATION_LENGTH;

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
            break;
        }

        // 鍔犲叆 IMU 鏁版嵁
        // Add new imu data to preintegration
        preintegrationlist.back()->addNewImu(imu_cur);

        imu_pre = imu_cur;
        imu_cur = imufile.next();

        if (imu_cur.time > sow) {
            // 鍔犲叆 GNSS 骞惰鍙栨柊鐨?GNSS
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
                gnsslist.push_back(gnss);

                gnss = gnssfile.next();
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold)) {
                    gnss = gnssfile.next();
                }

                // GNSS 涓柇閰嶇疆
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

            // IMU 鎻掑€煎鐞?            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) {
            } else if (isneed == 1) {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = imufile.next();
            } else if (isneed == 2) {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }

            // 涓嬩竴涓椂闂磋妭鐐?            // next time node
            timelist.push_back(sow);
            sow += INTEGRATION_LENGTH;

            // 褰撳墠鏃跺埢鐘舵€佸姞鍏ュ埌婊戠獥
            state_curr                               = preintegrationlist.back()->currentState();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Preintegration::stateToData(state_curr, preintegration_options);

            // 鏋勫缓浼樺寲闂
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

                // 鍙傛暟鍧?                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    // 浣嶅Э
                    ceres::Manifold *manifold = new PoseManifold();
                    problem.AddParameterBlock(statedatalist[k].pose, Preintegration::numPoseParameter(), manifold);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Preintegration::numMixParameter(preintegration_options));
                }

                // GNSS 鍥犲瓙
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

                // 棰勭Н鍒嗗洜瀛?                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                {
                    // IMU 璇樊绾︽潫
                    // add IMU bias-constraint factors
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // 鍏堥獙鍥犲瓙
                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                // 姹傝В鏈€灏忎簩涔?                // solve the Least-Squares problem
                options.max_num_iterations = num_iterations / 4;
                solver.Solve(options, &problem, &summary);

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

                // 杈撳嚭杩涘害
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
                    // 杈圭紭鍖?                    // marginalization
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

                    // 棰勭Н鍒嗗洜瀛?                    // preintegration factors
                    {
                        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS 鍥犲瓙
                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    // 杩涜杈圭紭鍖?                    // 杩涜杈圭紭鍖?                    // do marginalization
                    marginalization_info->marginalization();

                    // 鏁版嵁鎸囬拡閲嶆槧灏?                    // 鏁版嵁鎸囬拡閲嶆槧灏?                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                }

                // 婊戠獥澶勭悊
                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time)) {
                        gnsslist.pop_front();
                    }
                    timelist.pop_front();
                    preintegrationlist.pop_front();

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

            // 鏋勫缓鏂扮殑棰勭Н鍒?            // build a new preintegration object
            preintegrationlist.emplace_back(
                Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
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

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 鎺ヨ繎鍓嶄竴鍘嗗厓
        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // 鎺ヨ繎鍚庝竴鍘嗗厓
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        // 闇€瑕佹彃鍊?        // need interpolation
        return 2;
    }

    return 0;
}



