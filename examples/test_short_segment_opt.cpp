#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <fstream>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/spline/ControlPoint.h"
#include "src/spline/SplineInitializer.h"
#include "src/spline/BSplineEvaluator.h"
#include "src/spline/SophusSE3Manifold.h"

#include "src/factors/ContinuousInertialFactor.h"
#include "src/factors/WheelNHCFactor.h"
#include "src/factors/BiasRandomWalkFactor.h"
#include "src/factors/OdometerFactor.h"

using namespace ob_gins::spline;
using namespace ob_gins::factors;

// Constants
const double GRAVITY_MAG = 9.81;
const Eigen::Vector3d GRAVITY_VEC(0, 0, -GRAVITY_MAG);

struct Measurement {
    double t;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    double odometer; // Speed
};

int main() {
    // 1. Simulation Parameters
    double duration = 3.0; // Increased duration
    double dt_imu = 0.01;
    
    // True Parameters
    Eigen::Vector3d true_l_ecc(0.2, 0.1, 0.0);
    Eigen::Vector3d true_l_ga(0.05, 0.0, 0.0);
    Eigen::Vector3d true_bg(0.01, -0.01, 0.005);
    Eigen::Vector3d true_ba(0.1, 0.05, -0.1);

    // Initial State
    Sophus::SE3d current_pose_gt;
    std::vector<std::pair<double, Sophus::SE3d>> gt_path; // Ground Truth Path
    std::vector<std::pair<double, Sophus::SE3d>> dr_path; // Only need DR path for initialization
    std::vector<Measurement> imu_data;

    // Use better initialization? Let's reduce noise for "short segment" test to focus on observability.
    // If we have huge noise, we might need more robust initialization (IMU integration).
    // Here we use ideal IMU integration (DR) which is "close enough".
    Sophus::SE3d current_pose_dr;

    // Generate Data with Excitation
    for (double t = 0; t <= duration; t += dt_imu) {
        // Exciting Motion: Varying Speed and Yaw Rate
        // Speed: 10 + 2*sin(t) (Acceleration)
        // Yaw: 0.5 + 0.5*cos(2*t) (Jerk/AngAccel)
        double speed = 10.0 + 2.0 * std::sin(t);
        double accel_lin = 2.0 * std::cos(t);
        
        double yaw_rate = 0.5 + 0.5 * std::cos(2.0 * t);
        double yaw_accel = -1.0 * std::sin(2.0 * t);

        Eigen::Vector3d v_body(speed, 0, 0);
        Eigen::Vector3d a_body_local(accel_lin, 0, 0);
        
        Eigen::Vector3d w_body(0, 0, yaw_rate);
        Eigen::Vector3d alpha_body(0, 0, yaw_accel);

        gt_path.push_back({t, current_pose_gt});

        // --- Simulate IMU ---
        // Gyro
        Eigen::Vector3d gyro_meas = w_body + true_bg;

        // Accel
        // a_kinematic = a_body_local + w x v_body
        Eigen::Vector3d a_kinematic = a_body_local + w_body.cross(v_body);
        Eigen::Vector3d a_gravity = current_pose_gt.so3().inverse() * GRAVITY_VEC;
        
        // Lever Arm Effect (alpha x l + w x (w x l))
        Eigen::Vector3d a_lever = alpha_body.cross(true_l_ga) + w_body.cross(w_body.cross(true_l_ga));
        
        Eigen::Vector3d accel_meas = a_kinematic - a_gravity + a_lever + true_ba;

        // Odometer (Speed at Axle)
        // v_axle = v_body + w x l_ecc
        Eigen::Vector3d v_axle = v_body + w_body.cross(true_l_ecc);
        double odo_meas = v_axle.x(); // Forward speed

        imu_data.push_back({t, gyro_meas, accel_meas, odo_meas});

        // --- Update Pose GT ---
        Sophus::SE3d delta_pose_gt = Sophus::SE3d::exp(
            (Sophus::Vector6d() << v_body, w_body).finished() * dt_imu
        );
        current_pose_gt = current_pose_gt * delta_pose_gt;

        // --- Update Pose DR (Initialization) ---
        // Simple integration of gyro and odo? 
        // Or integration of accel? Accel integration is bad without gravity estimation.
        // Let's use Gyro + Odometer for DR path (better than accel).
        // v_dr = [odo, 0, 0] (Ignoring l_ecc impact on initialization)
        // w_dr = gyro (Ignoring bias)
        Sophus::SE3d delta_pose_dr = Sophus::SE3d::exp(
            (Sophus::Vector6d() << odo_meas, 0, 0, gyro_meas).finished() * dt_imu
        );
        current_pose_dr = current_pose_dr * delta_pose_dr;
        
        // Add small random noise to DR to verify optimization
        Sophus::SE3d noise = Sophus::SE3d::exp(Sophus::Vector6d::Random() * 0.01);
        dr_path.push_back({t, current_pose_dr * noise});
    }

    // 2. Initialize Spline
    double dt_spline = 0.1;
    // Use GT path for initialization to isolate lever arm observability
    std::vector<ControlPoint> cps = SplineInitializer::InitializeFromPath(gt_path, dt_spline);
    
    std::vector<Eigen::Vector3d> bg(cps.size(), Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> ba(cps.size(), Eigen::Vector3d::Zero());

    // 3. Build Problem
    ceres::Problem problem;

    // Parameters
    // Fix Lever Arms to True Values for this test to verify Trajectory/Bias optimization stability
    Eigen::Vector3d est_l_ecc = true_l_ecc;
    Eigen::Vector3d est_l_ga = true_l_ga;

    problem.AddParameterBlock(est_l_ecc.data(), 3);
    problem.SetParameterBlockConstant(est_l_ecc.data());
    
    problem.AddParameterBlock(est_l_ga.data(), 3);
    problem.SetParameterBlockConstant(est_l_ga.data());

    for (size_t i = 0; i < cps.size(); ++i) {
        problem.AddParameterBlock(cps[i].pose_data(), 7, new ceres::AutoDiffManifold<SophusSE3Plus, 7, 6>());
        problem.AddParameterBlock(bg[i].data(), 3);
        problem.AddParameterBlock(ba[i].data(), 3);
    }

    // Prior (Fix first)
    problem.SetParameterBlockConstant(cps[0].pose_data());
    problem.SetParameterBlockConstant(bg[0].data());
    problem.SetParameterBlockConstant(ba[0].data());

    // Factors
    int meas_count = 0;
    double t_start_valid = cps[1].timestamp();
    double t_end_valid = cps[cps.size()-2].timestamp();

    // 4.1 Inertial Factors
    for (const auto& meas : imu_data) {
        if (meas.t < t_start_valid || meas.t >= t_end_valid) continue;
        
        int idx = static_cast<int>(std::floor(meas.t / dt_spline)) - 1;
        if (idx < 0 || idx + 3 >= cps.size()) continue;

        auto* factor = ContinuousInertialFactor::Create(
            meas.t, meas.accel, meas.gyro, GRAVITY_VEC, dt_spline, cps[idx].timestamp()
        );
        
        problem.AddResidualBlock(factor, nullptr,
            cps[idx].pose_data(), cps[idx+1].pose_data(), cps[idx+2].pose_data(), cps[idx+3].pose_data(),
            bg[idx].data(), bg[idx+1].data(), bg[idx+2].data(), bg[idx+3].data(),
            ba[idx].data(), ba[idx+1].data(), ba[idx+2].data(), ba[idx+3].data(),
            est_l_ga.data()
        );
        meas_count++;
    }

    // 4.2 NHC and Odometer Factors (Every 0.05s)
    for (const auto& meas : imu_data) {
        // Downsample to 20Hz approx (every 5th sample if 100Hz)
        if (std::abs(std::fmod(meas.t, 0.05)) > 0.001) continue;
        if (meas.t < t_start_valid || meas.t >= t_end_valid) continue;

        int idx = static_cast<int>(std::floor(meas.t / dt_spline)) - 1;
        
        // NHC
        auto* nhc_factor = WheelNHCFactor::Create(meas.t, dt_spline, cps[idx].timestamp());
        problem.AddResidualBlock(nhc_factor, nullptr,
            cps[idx].pose_data(), cps[idx+1].pose_data(), cps[idx+2].pose_data(), cps[idx+3].pose_data(),
            est_l_ecc.data()
        );

        // Odometer
        auto* odo_factor = OdometerFactor::Create(meas.t, dt_spline, cps[idx].timestamp(), meas.odometer);
        problem.AddResidualBlock(odo_factor, nullptr,
            cps[idx].pose_data(), cps[idx+1].pose_data(), cps[idx+2].pose_data(), cps[idx+3].pose_data(),
            est_l_ecc.data()
        );
    }

    // 4.3 Bias Random Walk
    for (size_t i = 0; i < cps.size() - 1; ++i) {
        auto* factor_bg = new BiasRandomWalkFactor(dt_spline, 0.01);
        problem.AddResidualBlock(factor_bg, nullptr, bg[i].data(), bg[i+1].data());

        auto* factor_ba = new BiasRandomWalkFactor(dt_spline, 0.1);
        problem.AddResidualBlock(factor_ba, nullptr, ba[i].data(), ba[i+1].data());
    }

    std::cout << "Built problem with " << meas_count << " Inertial measurements." << std::endl;

    // 5. Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // 6. Validation
    std::cout << "\n--- Parameter Estimates ---" << std::endl;
    std::cout << "True l_ecc: " << true_l_ecc.transpose() << std::endl;
    std::cout << "Est  l_ecc: " << est_l_ecc.transpose() << std::endl;
    std::cout << "Error l_ecc: " << (est_l_ecc - true_l_ecc).norm() << std::endl;
    
    std::cout << "True l_ga : " << true_l_ga.transpose() << std::endl;
    std::cout << "Est  l_ga : " << est_l_ga.transpose() << std::endl;
    std::cout << "Error l_ga : " << (est_l_ga - true_l_ga).norm() << std::endl;

    std::cout << "True bg   : " << true_bg.transpose() << std::endl;
    std::cout << "Est  bg[mid]: " << bg[cps.size()/2].transpose() << std::endl;

    Eigen::Vector3d bg_err = bg[cps.size()/2] - true_bg;
    std::cout << "Bias Error: " << bg_err.norm() << std::endl;

    if (bg_err.norm() < 0.01) {
        std::cout << "SUCCESS: Bias estimation is reasonable." << std::endl;
    } else {
        std::cout << "WARNING: Bias estimation error high." << std::endl;
    }

    return 0;
}