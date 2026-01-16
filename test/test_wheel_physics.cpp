#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "src/spline/ControlPoint.h"
#include "src/spline/BSplineEvaluator.h"

using namespace ob_gins::spline;

void assert_near(const std::string& name, double val, double target, double tol = 1e-3) {
    double err = std::abs(val - target);
    std::cout << std::left << std::setw(30) << name 
              << " | Val: " << std::setw(10) << val 
              << " Target: " << std::setw(10) << target
              << " Error: " << std::setw(10) << err 
              << (err < tol ? " [PASS]" : " [FAIL]") << std::endl;
    if (err >= tol) exit(1);
}

void assert_vec_near(const std::string& name, const Eigen::Vector3d& val, const Eigen::Vector3d& target, double tol = 1e-3) {
    double err = (val - target).norm();
    std::cout << std::left << std::setw(30) << name 
              << " | Val: [" << val.transpose() << "] "
              << " Target: [" << target.transpose() << "] "
              << " Error: " << err 
              << (err < tol ? " [PASS]" : " [FAIL]") << std::endl;
    if (err >= tol) exit(1);
}

int main() {
    // 1. Setup Parameters
    double dt_spline = 0.1;
    double omega_target = 10.0; // rad/s
    
    // 2. Initialize Control Points
    // Create 4 points representing constant rotation around Z
    std::vector<ControlPoint> control_points;
    for (int i = 0; i < 4; ++i) {
        double t = i * dt_spline;
        double yaw = omega_target * t;
        
        // Construct pose: R = RotZ(yaw), p = (0,0,0)
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Sophus::SE3d pose(Eigen::Quaterniond(R), Eigen::Vector3d::Zero());
        
        control_points.emplace_back(t, pose);
    }

    // 3. Evaluate at midpoint t = 0.15 (u = 0.5)
    // Segment 0 covers [0.1, 0.2) using CP0..CP3
    double t_query = 0.15;
    double u = (t_query - control_points[1].timestamp()) / dt_spline; // (0.15 - 0.1)/0.1 = 0.5

    auto res = BSplineEvaluator::Evaluate(
        u, dt_spline, 
        control_points[0], control_points[1], control_points[2], control_points[3]
    );

    std::cout << "--- Kinematics Verification ---" << std::endl;
    // Check Angular Velocity
    assert_vec_near("Angular Velocity (Body)", res.w_body, Eigen::Vector3d(0, 0, omega_target));
    
    // Check Angular Acceleration (Should be 0 for constant rotation)
    assert_vec_near("Angular Accel (Body)", res.alpha_body, Eigen::Vector3d::Zero());

    // Check Linear Acceleration of Center (Should be 0 as position is fixed)
    assert_vec_near("Linear Accel Center (World)", res.a_world, Eigen::Vector3d::Zero());


    std::cout << "\n--- Centrifugal Force Verification ---" << std::endl;
    // 4. Calculate Lever Arm Acceleration
    Eigen::Vector3d l_ecc_body(0.3, 0.0, 0.0); // 0.3m on X axis

    // Formula: a_point = a_center_body + alpha x l + w x (w x l)
    // Note: BSplineEvaluator returns a_world for the center. We need a_center_body.
    // a_center_body = R^T * a_world
    Eigen::Vector3d a_center_body = res.pose.so3().inverse() * res.a_world;
    
    Eigen::Vector3d w = res.w_body;
    Eigen::Vector3d alpha = res.alpha_body;
    
    Eigen::Vector3d a_centrifugal = w.cross(w.cross(l_ecc_body));
    Eigen::Vector3d a_euler = alpha.cross(l_ecc_body);
    Eigen::Vector3d a_coriolis = Eigen::Vector3d::Zero(); // No relative velocity of point wrt body

    Eigen::Vector3d a_point_body = a_center_body + a_euler + a_centrifugal;

    std::cout << "Centrifugal Term: " << a_centrifugal.transpose() << std::endl;
    
    // Theoretical Expected:
    // w = [0, 0, 10]
    // l = [0.3, 0, 0]
    // w x l = [0, 3, 0]
    // w x (w x l) = [0, 0, 10] x [0, 3, 0] = [-30, 0, 0]
    Eigen::Vector3d expected_accel(-30.0, 0.0, 0.0);

    assert_vec_near("Point Acceleration", a_point_body, expected_accel, 1e-2);

    std::cout << "\nSUCCESS: Physics Verification Passed." << std::endl;

    // 5. Data Logging for Visualization
    std::ofstream file("wheel_physics_results.csv");
    file << "t,w_x,w_y,w_z,alpha_x,alpha_y,alpha_z,a_center_x,a_center_y,a_center_z,a_point_x,a_point_y,a_point_z\n";
    file << std::fixed << std::setprecision(6);

    double t_start = control_points[1].timestamp();     // 0.1
    double t_end = control_points[2].timestamp();       // 0.2
    double dt_log = 0.001; // 1ms resolution

    for (double t = t_start; t < t_end; t += dt_log) {
        double u_log = (t - t_start) / dt_spline;
        auto res_log = BSplineEvaluator::Evaluate(
            u_log, dt_spline, 
            control_points[0], control_points[1], control_points[2], control_points[3]
        );

        // Compute point acceleration again for logging
        Eigen::Vector3d a_c_body = res_log.pose.so3().inverse() * res_log.a_world;
        Eigen::Vector3d w_log = res_log.w_body;
        Eigen::Vector3d alpha_log = res_log.alpha_body;
        
        // Centrifugal + Euler
        Eigen::Vector3d a_cent = w_log.cross(w_log.cross(l_ecc_body));
        Eigen::Vector3d a_eul = alpha_log.cross(l_ecc_body);
        Eigen::Vector3d a_p_body = a_c_body + a_cent + a_eul;

        file << t << ","
             << w_log.x() << "," << w_log.y() << "," << w_log.z() << ","
             << alpha_log.x() << "," << alpha_log.y() << "," << alpha_log.z() << ","
             << a_c_body.x() << "," << a_c_body.y() << "," << a_c_body.z() << ","
             << a_p_body.x() << "," << a_p_body.y() << "," << a_p_body.z() << "\n";
    }
    file.close();
    std::cout << "Physics data saved to wheel_physics_results.csv" << std::endl;

    return 0;
}
