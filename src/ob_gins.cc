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
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);

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
    std::string imupath    = config["imufile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();
    int imudatalen         = config["imudatalen"].as<int>();
    int imudatarate        = config["imudatarate"].as<int>();

    // Consider Earth's rotation in mechanization (Coriolis and Earth-rate effects)
    // Affects preintegration dynamics if true
    bool isearth = config["isearth"].as<bool>();

    GnssFileLoader gnssfile(gnsspath);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
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
    parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
    parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;

    bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;

    // Standalone ODO file (optional; required if isuseodo is true)
    OdoFileLoader *odofile_ptr = nullptr;
    YAML::Node odo_cfg         = config["odometer"];
    if (isuseodo) {
        if (!odo_cfg["file"]) {
            std::cout << "Odometer enabled but no odometer.file specified in config" << std::endl;
            return -1;
        }
        std::string odopath = odo_cfg["file"].as<std::string>();
        int odocolumns      = odo_cfg["columns"] ? odo_cfg["columns"].as<int>() : 2;
        odofile_ptr         = new OdoFileLoader(odopath, odocolumns);
        if (!odofile_ptr->isOpen()) {
            std::cout << "Failed to open odometer file" << std::endl;
            return -1;
        }
        // Optional overrides for ODO installation parameters (position/orientation)
        // lever in meters, odoangle in degrees
        if (odo_cfg["lever"]) {
            auto lv = odo_cfg["lever"].as<std::vector<double>>();
            if (lv.size() == 3) {
                parameters->lodo = Vector3d(lv.data());
            }
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
    if (odo_cfg["factor"]) {
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

    // GNSS浠跨湡涓柇閰嶇疆
    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // Data alignment: advance IMU and GNSS streams to starttime
    // Read until first sample >= starttime for each stream
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur;
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    // Align ODO stream to start time
    std::deque<ODO> odolist;
    ODO odo;
    if (isuseodo) {
        odo = odofile_ptr->next();
        while (odo.time < starttime && !odofile_ptr->isEof()) {
            odo = odofile_ptr->next();
        }
    }

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

    statelist[0]     = state_curr;
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // Initial preintegration: seed with current IMU sample and initial state
    // Initial preintegration
    preintegrationlist.emplace_back(
        Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));

    // Read next GNSS epoch for subsequent steps
    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh            = Earth::global2local(station_origin, gnss.blh);

    // Marginalization bookkeeping (previous prior and parameter blocks)
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    // Move to next integration node (advance target time by INTEGRATION_LENGTH)
    sow += INTEGRATION_LENGTH;

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
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

        imu_pre = imu_cur;
        imu_cur = imufile.next();

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
                imu_cur = imufile.next();
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

                // Add wheel-speed (odometer) factors as a separate measurement stream
                if (isuseodo) {
                    // Advance ODO buffer up to the latest node time
                    while (odofile_ptr && !odofile_ptr->isEof() && odo.time <= *timelist.rbegin()) {
                        odolist.push_back(odo);
                        odo = odofile_ptr->next();
                    }

                    // Remove outdated ODO measurements beyond the window
                    while (!odolist.empty() && odolist.front().time < timelist.front() - 0.5 * INTEGRATION_LENGTH) {
                        odolist.pop_front();
                    }

                    // Add ODO residuals by mapping to the nearest node
                    const int mix_dim   = Preintegration::numMixParameter(preintegration_options);
                    const Vector3d lodo = parameters->lodo;                               // lever arm (body)
                    ceres::LossFunction *odo_loss = new ceres::HuberLoss(odo_huber_delta);

                    static double sodo_param = 1.0; // global scale
                    problem.AddParameterBlock(&sodo_param, 1);
                    problem.AddResidualBlock(new SodoPriorFactor(1.0, sodo_prior_sigma), nullptr, &sodo_param);

                    // Global calibration for R_b^v as delta angles (roll,pitch,yaw) around config bodyangle
                    static double rbw_param[3] = {0.0, 0.0, 0.0};
                    problem.AddParameterBlock(rbw_param, 3);
                    problem.AddResidualBlock(new AnglesPriorFactor(rbw_prior_sigma_rad), nullptr, rbw_param);

                    for (const auto &m : odolist) {
                        if (m.time < timelist.front() - MINIMUM_INTERVAL || m.time > timelist.back() + MINIMUM_INTERVAL) {
                            continue;
                        }

                        size_t nearest = 0;
                        double best    = 1e9;
                        for (size_t i = 0; i < timelist.size(); ++i) {
                            double d = fabs(m.time - timelist[i]);
                            if (d < best) {
                                best    = d;
                                nearest = i;
                            }
                        }

                        // Extract more precise omega and acc near this measurement time
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
                            // fallback to last at-nearest node
                            omega_b = (omega_nodes.size() > nearest) ? omega_nodes[nearest] : Vector3d::Zero();
                        }

                        // Adaptive downweighting: low-speed, high yaw-rate, high acceleration
                        double sigma = parameters->odo_std[0];
                        if (fabs(m.vel) < odo_low_speed_thresh) sigma *= odo_low_speed_scale;         // near standstill
                        if (fabs(omega_b.z()) > odo_yaw_rate_thresh) sigma *= odo_yaw_rate_scale;     // aggressive turn
                        if (acc_b.norm() > odo_accel_thresh) sigma *= odo_accel_scale;                // harsh accel/brake

                    // Use odoangle as base if provided; otherwise fallback to bodyangle
                    Vector3d odo_base_angle = bodyangle;
                    if (odo_cfg["odoangle"]) {
                        auto ang = odo_cfg["odoangle"].as<std::vector<double>>();
                        if (ang.size() == 3) {
                            odo_base_angle = Vector3d(ang.data()) * D2R;
                        }
                    }

                    Vector3d omega_x_l = omega_b.cross(lodo);
                    auto factor        = new OdoFactor(m.vel, sigma, omega_x_l, mix_dim, odo_base_angle);
                        problem.AddResidualBlock(factor, odo_loss,
                                                 statedatalist[nearest].pose,
                                                 statedatalist[nearest].mix,
                                                 &sodo_param,
                                                 rbw_param);
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
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    gnssfile.close();
    if (odofile_ptr) {
        delete odofile_ptr;
        odofile_ptr = nullptr;
    }

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
