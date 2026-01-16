#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "src/spline/ControlPoint.h"
#include "src/factors/WheelNHCFactor.h"

using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main() {
    // 1. Setup Parameters
    double dt_spline = 0.1;
    double v_target = 10.0; // m/s
    double w_target = 1.0;  // rad/s
    
    // Circular motion
    // R = 10m. 
    // Center (0, 10).
    // Pos(t): x = 10*sin(w*t), y = 10*(1-cos(w*t))
    // Vel(t): vx = 10*w*cos, vy = 10*w*sin.
    // Yaw(t): w*t
    
    // 2. Initialize Control Points
    std::vector<ControlPoint> control_points;
    for (int i = 0; i < 4; ++i) {
        double t = i * dt_spline;
        double yaw = w_target * t;
        
        Eigen::Vector3d trans(10.0 * std::sin(yaw), 10.0 * (1.0 - std::cos(yaw)), 0.0);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Sophus::SE3d pose(Eigen::Quaterniond(R), trans);
        
        control_points.emplace_back(t, pose);
    }

    // 3. Setup Lever Arm
    // We want a lever arm such that v_axle has 0 lateral/vertical velocity.
    // In this pure circular motion of the body origin, v_body is [10, 0, 0].
    // If we add a lever arm l = [0, 0.5, 0], then:
    // v_axle = v_body + w x l = [10, 0, 0] + [0, 0, 1] x [0, 0.5, 0]
    //        = [10, 0, 0] + [-0.5, 0, 0] = [9.5, 0, 0].
    // Lateral is still 0.
    // If we pick l = [0.5, 0, 0], w x l = [0, 0.5, 0]. v_axle = [10, 0.5, 0]. Lateral is 0.5.
    // This would VIOLATE NHC.
    
    // Case 1: Lever Arm = [0, 0.5, 0]. Expect Residuals ~ 0.
    Eigen::Vector3d l_ecc_1(0.0, 0.5, 0.0);
    
    // Case 2: Lever Arm = [0.5, 0, 0]. Expect Residual y != 0.
    Eigen::Vector3d l_ecc_2(0.5, 0.0, 0.0);

    // 4. Evaluate Factor
    double t_query = 0.15;
    
    WheelNHCFactor factor(t_query, dt_spline, control_points[0].timestamp());
    
    double residuals[2];
    double* params_1[5] = {
        control_points[0].pose_data(),
        control_points[1].pose_data(),
        control_points[2].pose_data(),
        control_points[3].pose_data(),
        l_ecc_1.data()
    };

    factor(params_1[0], params_1[1], params_1[2], params_1[3], params_1[4], residuals);

    std::cout << "--- Case 1: Lever Arm [0, 0.5, 0] (Should satisfy NHC) ---" << std::endl;
    std::cout << "Residuals: " << residuals[0] << ", " << residuals[1] << std::endl;
    
    if (std::abs(residuals[0]) < 1e-2 && std::abs(residuals[1]) < 1e-2) {
        std::cout << "PASS: Residuals near zero." << std::endl;
    } else {
        std::cout << "FAIL: Residuals too high." << std::endl;
        exit(1);
    }

    double* params_2[5] = {
        control_points[0].pose_data(),
        control_points[1].pose_data(),
        control_points[2].pose_data(),
        control_points[3].pose_data(),
        l_ecc_2.data()
    };

    factor(params_2[0], params_2[1], params_2[2], params_2[3], params_2[4], residuals);
    
    std::cout << "\n--- Case 2: Lever Arm [0.5, 0, 0] (Should violate NHC y) ---" << std::endl;
    std::cout << "Residuals: " << residuals[0] << ", " << residuals[1] << std::endl;

    // Expected violation:
    // v_y_expected = 0.5. Scaled by 10.0 -> 5.0.
    if (std::abs(residuals[0] - 5.0) < 0.5) { // Tolerance for spline approximation
        std::cout << "PASS: Residual matches expected violation (approx 5.0)." << std::endl;
    } else {
        std::cout << "FAIL: Residual " << residuals[0] << " != expected ~5.0" << std::endl;
        exit(1);
    }

    return 0;
}
