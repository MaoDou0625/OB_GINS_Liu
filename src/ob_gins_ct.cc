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

using namespace ob_gins;
using namespace ob_gins::spline;
using namespace ob_gins::factors;

// Helper to find CP index for time t
// Assumes uniform spline with spacing dt starting at t0
int findControlPointIndex(double t, double t0, double dt, int max_idx) {
    // We assume the spline is defined such that evaluation at t uses:
    // P(u), where u = (t - t0)/dt - k
    // In BSplineEvaluator, t_val should be in [t_k + dt, t_k + 2dt) for CPs k..k+3?
    // Let's rely on the formula: k = floor((t - t0 - dt) / dt)
    // t0 is the timestamp of the first control point.
    return static_cast<int>(std::floor((t - dt - t0) / dt));
}

struct LeverArmPriorFactor {
    LeverArmPriorFactor(const Eigen::Vector3d& prior, double sigma)
        : prior_(prior), sigma_(sigma) {}

    template <typename T>
    bool operator()(const T* const l_ptr, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> l(l_ptr);
        
        residuals[0] = (l[0] - T(prior_[0])) / T(sigma_);
        residuals[1] = (l[1] - T(prior_[1])) / T(sigma_);
        residuals[2] = (l[2] - T(prior_[2])) / T(sigma_);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& prior, double sigma) {
        return new ceres::AutoDiffCostFunction<LeverArmPriorFactor, 3, 3>(
            new LeverArmPriorFactor(prior, sigma));
    }

    Eigen::Vector3d prior_;
    double sigma_;
};

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

    std::string gnss_path = config["gnssfile"].as<std::string>();
    std::string imu_path;
    if (config["imu_main"]) {
        imu_path = config["imu_main"]["file"].as<std::string>();
    } else {
        LOG(ERROR) << "No imu_main configuration found.";
        return -1;
    }
    
    std::string output_path = config["outputpath"].as<std::string>();
    if (!std::filesystem::exists(output_path)) {
        std::filesystem::create_directories(output_path);
    }

    // 2. Load Data
    LOG(INFO) << "Loading data...";
    ImuFileLoader imu_loader(imu_path, 7, 200);
    GnssFileLoader gnss_loader(gnss_path);

    std::vector<IMU> imu_data;
    while (!imu_loader.isEof()) {
        imu_data.push_back(imu_loader.next());
    }

    std::vector<GNSS> gnss_data;
    while (!gnss_loader.isEof()) {
        gnss_data.push_back(gnss_loader.next());
    }

    if (imu_data.empty() || gnss_data.empty()) {
        LOG(ERROR) << "Empty data loaded.";
        return -1;
    }

    double t_start = std::max(imu_data.front().time, gnss_data.front().time);
    double t_end = std::min(imu_data.back().time, gnss_data.back().time);

    if (config["starttime"]) t_start = std::max(t_start, config["starttime"].as<double>());
    if (config["endtime"]) t_end = std::min(t_end, config["endtime"].as<double>());

    LOG(INFO) << "Time window: " << std::fixed << t_start << " to " << t_end 
              << " (Duration: " << (t_end - t_start) << "s)";

    std::vector<IMU> valid_imu;
    valid_imu.reserve(imu_data.size());
    for (const auto& imu : imu_data) {
        if (imu.time >= t_start && imu.time <= t_end) {
            valid_imu.push_back(imu);
        }
    }

    GNSS origin_gnss;
    bool origin_set = false;
    std::vector<GNSS> valid_gnss;
    
    for (const auto& gnss : gnss_data) {
        if (gnss.time >= t_start) {
            if (!origin_set) {
                origin_gnss = gnss;
                origin_set = true;
            }
            if (gnss.time <= t_end) {
                valid_gnss.push_back(gnss);
            }
        }
    }
    
    if (!origin_set || valid_gnss.empty()) {
        LOG(ERROR) << "No valid GNSS data in time window.";
        return -1;
    }

    Vector3d station_origin = origin_gnss.blh;
    LOG(INFO) << "Origin set at: " << station_origin.transpose();

    // Calculate local gravity
    // Earth::gravity returns positive scalar g.
    // In NED (North-East-Down), gravity vector is [0, 0, g].
    // If the system is ENU (East-North-Up), gravity vector is [0, 0, -g].
    // This project seems to use ENU for local coordinates (based on -9.8 hardcode previously).
    double g_mag = Earth::gravity(station_origin);
    Vector3d gravity_vec(0, 0, -g_mag);
    LOG(INFO) << "Local gravity magnitude: " << g_mag << " -> Vector: " << gravity_vec.transpose();

    // Load Calibration & Noise
    bool optimize_leverarm = false;
    Vector3d initial_l_gnss = Vector3d::Zero();
    
    if (config["calibration"]) {
        optimize_leverarm = config["calibration"]["optimize_leverarm"].as<bool>();
        if (config["calibration"]["T_body_gnss"]) {
            std::vector<double> T_vec = config["calibration"]["T_body_gnss"].as<std::vector<double>>();
            if (T_vec.size() == 16) {
                // T_body_gnss is 4x4, translation is at indices 3, 7, 11
                // Row-major:
                // 0  1  2  3
                // 4  5  6  7
                // 8  9 10 11
                initial_l_gnss << T_vec[3], T_vec[7], T_vec[11];
            }
        }
    }

    double acc_noise = 1.0e-2;
    double gyr_noise = 1.0e-3;
    double acc_bias_rw = 1.0e-4;
    double gyr_bias_rw = 1.0e-5;

    if (config["imu_noise"]) {
        acc_noise = config["imu_noise"]["accel_noise"].as<double>();
        gyr_noise = config["imu_noise"]["gyro_noise"].as<double>();
        acc_bias_rw = config["imu_noise"]["accel_bias_rw"].as<double>();
        gyr_bias_rw = config["imu_noise"]["gyro_bias_rw"].as<double>();
    }
    LOG(INFO) << "Noise Params: Acc " << acc_noise << " Gyr " << gyr_noise 
              << " AccRW " << acc_bias_rw << " GyrRW " << gyr_bias_rw;

    for (auto& gnss : valid_gnss) {
        gnss.blh = Earth::global2local(station_origin, gnss.blh);
    }

    // 3. Spline Initialization
    LOG(INFO) << "Initializing B-Spline...";
    double spline_dt = 0.1;
    
    std::vector<std::pair<double, Sophus::SE3d>> init_path;
    
    for (size_t i = 0; i < valid_gnss.size(); ++i) {
        const auto& curr = valid_gnss[i];
        
        Vector3d pos = curr.blh;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        
        if (i + 1 < valid_gnss.size()) {
            Vector3d delta = valid_gnss[i+1].blh - pos;
            if (delta.norm() > 1e-3) {
                Vector3d forward = delta.normalized();
                Vector3d up = Vector3d::UnitZ();
                Vector3d right = up.cross(forward).normalized();
                if (right.norm() < 1e-3) {
                    right = Vector3d::UnitY();
                }
                Vector3d ortho_up = forward.cross(right);
                
                Matrix3d R;
                R.col(0) = forward;
                R.col(1) = right;
                R.col(2) = ortho_up;
                q = Eigen::Quaterniond(R);
            }
        } else if (!init_path.empty()) {
            q = init_path.back().second.unit_quaternion();
        }

        init_path.push_back({curr.time, Sophus::SE3d(q, pos)});
    }
    
    std::vector<ControlPoint> control_points = 
        SplineInitializer::InitializeFromPath(init_path, spline_dt);
        
    if (control_points.empty()) {
        LOG(ERROR) << "Failed to initialize spline.";
        return -1;
    }
    
    double t0_spline = control_points.front().timestamp();
        LOG(INFO) << "Initialized " << control_points.size() << " control points.";
        LOG(INFO) << "Spline start time: " << t0_spline;
        
        // Debug: Check initial CP positions
        for(size_t i=0; i<std::min((size_t)5, control_points.size()); ++i) {
            LOG(INFO) << "Init CP[" << i << "] t=" << control_points[i].timestamp() 
                      << " Pos: " << control_points[i].pose().translation().transpose();
        }
    
        // 4. Build Factor Graph        LOG(INFO) << "Building Factor Graph...";
        ceres::Problem problem;
        auto* se3_manifold = new SophusSE3Manifold();
    
        std::vector<ceres::ResidualBlockId> imu_residual_ids;
        std::vector<ceres::ResidualBlockId> gnss_residual_ids;
    
        for (size_t i = 0; i < control_points.size(); ++i) {
            // Add Pose Param (SE3)
            problem.AddParameterBlock(control_points[i].pose_data(), 7, se3_manifold);
            
            // Add Bias Params (Vector3)
            problem.AddParameterBlock(control_points[i].bg_data(), 3);
            problem.AddParameterBlock(control_points[i].ba_data(), 3);
            
            if (i > 0) {
                double dt = spline_dt;
                
                auto* factor_g = new BiasRandomWalkFactor(dt, gyr_bias_rw);
                problem.AddResidualBlock(factor_g, nullptr, 
                    control_points[i-1].bg_data(), control_points[i].bg_data());
                
                auto* factor_a = new BiasRandomWalkFactor(dt, acc_bias_rw);
                problem.AddResidualBlock(factor_a, nullptr, 
                    control_points[i-1].ba_data(), control_points[i].ba_data());
            }
        }
        
        Vector3d l_gnss = initial_l_gnss;
        Vector3d l_imu = Vector3d::Zero(); 
    
        // 4.1 Add IMU Factors
        // gravity_vec is defined in main scope
        
        int added_imu = 0;
        for (const auto& imu : valid_imu) {
            int k = findControlPointIndex(imu.time, t0_spline, spline_dt, control_points.size());
            
            if (k < 0 || k + 3 >= (int)control_points.size()) continue;
    
            double dt = imu.dt;
            if (dt < 1e-6) continue;
    
            Vector3d gyro_rate = imu.dtheta / dt;
            Vector3d accel_rate = imu.dvel / dt;
            
            auto* factor = ContinuousInertialFactor::Create(
                imu.time, accel_rate, gyro_rate, gravity_vec, 
                spline_dt, control_points[k].timestamp(), acc_noise, gyr_noise
            );
            
            ceres::ResidualBlockId id = problem.AddResidualBlock(factor, nullptr, 
                control_points[k].pose_data(), control_points[k+1].pose_data(), 
                control_points[k+2].pose_data(), control_points[k+3].pose_data(),
                control_points[k].bg_data(), control_points[k+1].bg_data(), 
                control_points[k+2].bg_data(), control_points[k+3].bg_data(),
                control_points[k].ba_data(), control_points[k+1].ba_data(), 
                control_points[k+2].ba_data(), control_points[k+3].ba_data(),
                l_imu.data()
            );
            imu_residual_ids.push_back(id);
            added_imu++;
        }
        problem.AddParameterBlock(l_imu.data(), 3);
        problem.SetParameterBlockConstant(l_imu.data());
        LOG(INFO) << "Added " << added_imu << " IMU factors.";
    
        // 4.2 Add GNSS Factors
        int added_gnss = 0;
        
        for (const auto& gnss : valid_gnss) {
            int k = findControlPointIndex(gnss.time, t0_spline, spline_dt, control_points.size());
            if (k < 0 || k + 3 >= (int)control_points.size()) continue;
    
            Matrix3d cov = Matrix3d::Zero();
            cov.diagonal() = gnss.std.cwiseProduct(gnss.std);
            if (cov.determinant() < 1e-12) cov = Matrix3d::Identity() * 1.0;
            Eigen::LLT<Matrix3d> llt(cov.inverse());
            Matrix3d sqrt_info = llt.matrixL().transpose();
    
            auto* factor = ContinuousGnssFactor::Create(
                gnss.time, spline_dt, control_points[k].timestamp(), 
                gnss.blh, sqrt_info
            );
            
            ceres::ResidualBlockId id = problem.AddResidualBlock(factor, nullptr,
                control_points[k].pose_data(), control_points[k+1].pose_data(), 
                control_points[k+2].pose_data(), control_points[k+3].pose_data(),
                l_gnss.data()
            );
            gnss_residual_ids.push_back(id);
            added_gnss++;
        }
            problem.AddParameterBlock(l_gnss.data(), 3);
            if (!optimize_leverarm) {
                problem.SetParameterBlockConstant(l_gnss.data());
                } else {
                    // Add Prior to prevent unobservability (especially in Z direction for cars)
                    // Sigma = 0.01m (tight constraint to force bias estimation instead of lever arm drift)
                    auto* prior_factor = LeverArmPriorFactor::Create(initial_l_gnss, 0.01);
                    problem.AddResidualBlock(prior_factor, nullptr, l_gnss.data());
                    LOG(INFO) << "Added LeverArm Prior (sigma=0.01m)";
                }            LOG(INFO) << "Added " << added_gnss << " GNSS factors.";    
        auto EvaluateRMSE = [&](const std::string& label) {
            double cost_imu = 0, cost_gnss = 0;
            ceres::Problem::EvaluateOptions opt_imu;
            opt_imu.residual_blocks = imu_residual_ids;
            problem.Evaluate(opt_imu, &cost_imu, nullptr, nullptr, nullptr);
            
            ceres::Problem::EvaluateOptions opt_gnss;
            opt_gnss.residual_blocks = gnss_residual_ids;
            problem.Evaluate(opt_gnss, &cost_gnss, nullptr, nullptr, nullptr);
            
            double rmse_imu = std::sqrt(2.0 * cost_imu / (imu_residual_ids.size() * 6));
            double rmse_gnss = std::sqrt(2.0 * cost_gnss / (gnss_residual_ids.size() * 3)); // Assuming 3D position
            
            LOG(INFO) << label << " RMSE -> IMU: " << rmse_imu << " (norm), GNSS: " << rmse_gnss << " (weighted)";
        };
    
        EvaluateRMSE("Initial");
    
        // 5. Solve
        LOG(INFO) << "Starting Optimization...";
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 50;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 8;
    
        ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << summary.FullReport();

    LOG(INFO) << "Optimized Lever Arms:";
    LOG(INFO) << "  l_gnss (Body->Ant): " << l_gnss.transpose();
    LOG(INFO) << "  l_imu (Body->IMU):  " << l_imu.transpose();

    // 6. Export Result
        LOG(INFO) << "Exporting trajectory...";
        std::filesystem::path out_file = std::filesystem::path(output_path) / "OB_GINS_CT.txt";
        FILE* fp = fopen(out_file.string().c_str(), "w");
        if (!fp) {
            LOG(ERROR) << "Failed to open output file.";
            return -1;
        }
        
        fprintf(fp, "# Time Tx Ty Tz Qx Qy Qz Qw Vx Vy Vz Bgx Bgy Bgz Bax Bay Baz\n");
    
        double t_cursor = t_start;
        double sample_dt = 0.005; // 200Hz
    while (t_cursor <= t_end) {
        int k = findControlPointIndex(t_cursor, t0_spline, spline_dt, control_points.size());
        if (k < 0 || k + 3 >= (int)control_points.size()) {
            t_cursor += sample_dt;
            continue;
        }
        
        using T = double;
        T dt = spline_dt;
        T u = (t_cursor - (control_points[k].timestamp() + dt)) / dt;
        
        Eigen::Map<const Sophus::SE3d> T0(control_points[k].pose_data());
        Eigen::Map<const Sophus::SE3d> T1(control_points[k+1].pose_data());
        Eigen::Map<const Sophus::SE3d> T2(control_points[k+2].pose_data());
        Eigen::Map<const Sophus::SE3d> T3(control_points[k+3].pose_data());
        
        auto res = BSplineEvaluator::Evaluate<double>(u, dt, T0, T1, T2, T3);
        
        Vector3d pos_local = res.pose.translation();
        Vector3d vel_local = res.v_world;
        Quaterniond q = res.pose.unit_quaternion();
        
        // Linear Interpolate bias
        Vector3d bg = control_points[k].bg() * (1.0 - u) + control_points[k+1].bg() * u;
        Vector3d ba = control_points[k].ba() * (1.0 - u) + control_points[k+1].ba() * u;
        
        fprintf(fp, "%.4f %.4f %.4f %.4f %.6f %.6f %.6f %.6f %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.5f %.5f\n",
            t_cursor, 
            pos_local.x(), pos_local.y(), pos_local.z(),
            q.x(), q.y(), q.z(), q.w(),
            vel_local.x(), vel_local.y(), vel_local.z(),
            bg.x(), bg.y(), bg.z(),
            ba.x(), ba.y(), ba.z()
        );

        t_cursor += sample_dt;
    }
    
    fclose(fp);
    LOG(INFO) << "Done. Saved to " << out_file.string();

    // Export GNSS used (Local Frame) for comparison
    std::filesystem::path out_gnss = std::filesystem::path(output_path) / "OB_GINS_CT_GNSS.txt";
    FILE* fp_gnss = fopen(out_gnss.string().c_str(), "w");
    if (fp_gnss) {
        fprintf(fp_gnss, "# Time Tx Ty Tz\n");
        for (const auto& gnss : valid_gnss) {
            fprintf(fp_gnss, "%.4f %.4f %.4f %.4f\n", 
                gnss.time, gnss.blh.x(), gnss.blh.y(), gnss.blh.z());
        }
        fclose(fp_gnss);
        LOG(INFO) << "Saved processed GNSS to " << out_gnss.string();
    }

    return 0;
}
