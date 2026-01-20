#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>

#include "src/spline/ControlPoint.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/spline/SophusSE3Manifold.h"

using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main() {
    // 1. Setup Parameters
    double dt_spline = 0.05;
    double duration = 0.5; // Short duration is enough
    double alpha_z = 100.0; // rad/s^2
    
    Eigen::Vector3d true_l_ga(0.1, 0.0, 0.0);
    Eigen::Vector3d gravity(0.0, 0.0, 0.0); // Zero gravity to isolate lever arm

    // 2. Generate Control Points (Ground Truth Motion)
    // theta = 0.5 * alpha * t^2
    std::vector<ControlPoint> control_points;
    int num_cp = static_cast<int>(std::ceil(duration / dt_spline)) + 4;

    for (int i = 0; i < num_cp; ++i) {
        double t = i * dt_spline;
        double yaw = 0.5 * alpha_z * t * t;
        
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Sophus::SE3d pose(Eigen::Quaterniond(R), Eigen::Vector3d::Zero());
        
        control_points.emplace_back(t, pose);
    }
    
    // Biases (Fixed to Zero)
    std::vector<Eigen::Vector3d> bg(num_cp, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> ba(num_cp, Eigen::Vector3d::Zero());

    // 3. Build Problem
    ceres::Problem problem;
    
    // Parameter: Estimated Lever Arm
    Eigen::Vector3d estimated_l_ga(0.0, 0.0, 0.0);
    problem.AddParameterBlock(estimated_l_ga.data(), 3);

    // Parameters: Control Points & Biases (Fixed)
    for (int i = 0; i < num_cp; ++i) {
        problem.AddParameterBlock(control_points[i].pose_data(), 7, new SophusSE3Manifold());
        problem.SetParameterBlockConstant(control_points[i].pose_data());
        
        problem.AddParameterBlock(bg[i].data(), 3);
        problem.SetParameterBlockConstant(bg[i].data());
        
        problem.AddParameterBlock(ba[i].data(), 3);
        problem.SetParameterBlockConstant(ba[i].data());
    }

    // 4. Add Factors
    double dt_meas = 0.005; // 200Hz
    int meas_count = 0;

    double t_start_valid = control_points[1].timestamp();
    double t_end_valid = control_points[control_points.size()-2].timestamp();

    for (double t = 0; t <= duration; t += dt_meas) {
        if (t < t_start_valid || t >= t_end_valid) continue;
        
        // Find relevant control point
        int idx = static_cast<int>(std::floor(t / dt_spline)) - 1;
        if (idx < 0 || idx + 3 >= control_points.size()) continue;

        // Synthesize Measurements
        // w(t) = alpha * t
        Eigen::Vector3d w(0.0, 0.0, alpha_z * t);
        Eigen::Vector3d alpha(0.0, 0.0, alpha_z);
        
        Eigen::Vector3d gyro_meas = w;
        
        // a_meas = alpha x l + w x (w x l) (since kinematic accel is 0 and g is 0)
        Eigen::Vector3d accel_meas = alpha.cross(true_l_ga) + w.cross(w.cross(true_l_ga));
        
        ceres::CostFunction* cost_function = ContinuousInertialFactor::Create(
            t, accel_meas, gyro_meas, gravity, Eigen::Vector3d::Zero(),
            dt_spline, control_points[idx].timestamp(), 1.0, 1.0
        );

        problem.AddResidualBlock(cost_function, nullptr,
                                 control_points[idx].pose_data(),
                                 control_points[idx+1].pose_data(),
                                 control_points[idx+2].pose_data(),
                                 control_points[idx+3].pose_data(),
                                 bg[idx].data(), bg[idx+1].data(), bg[idx+2].data(), bg[idx+3].data(),
                                 ba[idx].data(), ba[idx+1].data(), ba[idx+2].data(), ba[idx+3].data(),
                                 estimated_l_ga.data());
        meas_count++;
    }
    std::cout << "Added " << meas_count << " measurements." << std::endl;

    // 5. Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << summary.BriefReport() << std::endl;

    // 6. Verify Result
    std::cout << "True Lever Arm: " << true_l_ga.transpose() << std::endl;
    std::cout << "Est Lever Arm : " << estimated_l_ga.transpose() << std::endl;

    // Log Results for Visualization
    std::ofstream file("inertial_physics_results.csv");
    file << "t,meas_ax,meas_ay,meas_az,pred_ax,pred_ay,pred_az\n";
    file << std::fixed << std::setprecision(6);

    for (double t = 0; t <= duration; t += dt_meas) {
        if (t < t_start_valid || t >= t_end_valid) continue;
        
        // Find relevant control point
        int idx = static_cast<int>(std::floor(t / dt_spline)) - 1;
        if (idx < 0 || idx + 3 >= control_points.size()) continue;

        // Re-evaluate spline
        double u = (t - control_points[idx+1].timestamp()) / dt_spline;
        auto res = BSplineEvaluator::Evaluate(
            u, dt_spline, 
            control_points[idx], control_points[idx+1], control_points[idx+2], control_points[idx+3]
        );

        // Physics Model with Estimated Lever Arm
        Eigen::Vector3d w = res.w_body;
        Eigen::Vector3d alpha = res.alpha_body;
        
        // Kinematic Accel at Gyro Center (Ideal)
        // a_world = R * a_body.
        // For this synthetic test, a_world (kinematic) is 0 because center is fixed.
        // So a_gyro = R^T * (0 - 0) = 0.
        // But let's use the rigorous formula if we had motion.
        // Here a_world_kinematic = 0. gravity = 0.
        Eigen::Vector3d a_gyro = Eigen::Vector3d::Zero();

        // Lever Arm Effect
        Eigen::Vector3d acc_lever = alpha.cross(estimated_l_ga) + w.cross(w.cross(estimated_l_ga));
        Eigen::Vector3d acc_pred = a_gyro + acc_lever; // + bias (0)

        // Measured (Synthetic Truth)
        Eigen::Vector3d w_true(0, 0, alpha_z * t);
        Eigen::Vector3d alpha_true(0, 0, alpha_z);
        Eigen::Vector3d acc_meas = alpha_true.cross(true_l_ga) + w_true.cross(w_true.cross(true_l_ga));

        file << t << ","
             << acc_meas.x() << "," << acc_meas.y() << "," << acc_meas.z() << ","
             << acc_pred.x() << "," << acc_pred.y() << "," << acc_pred.z() << "\n";
    }
    file.close();
    std::cout << "Results saved to inertial_physics_results.csv" << std::endl;

    Eigen::Vector3d error = estimated_l_ga - true_l_ga;
    if (error.norm() < 1e-3) {
        std::cout << "SUCCESS: Lever arm converged." << std::endl;
    } else {
        std::cout << "FAIL: Lever arm did not converge. Error: " << error.norm() << std::endl;
        exit(1);
    }

    return 0;
}
