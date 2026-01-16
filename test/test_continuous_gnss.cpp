#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "src/spline/ControlPoint.h"
#include "src/factors/ContinuousGnssFactor.h"

using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main() {
    // 1. Setup Parameters
    double dt_spline = 0.1;
    
    // 2. Initialize Control Points (Linear Motion along X, v=10 m/s)
    std::vector<ControlPoint> control_points;
    for (int i = 0; i < 4; ++i) {
        double t = i * dt_spline;
        
        Eigen::Vector3d trans(10.0 * t, 0.0, 0.0);
        Sophus::SE3d pose(Eigen::Quaterniond::Identity(), trans);
        
        control_points.emplace_back(t, pose);
    }

    // 3. Setup Lever Arm
    // l_ecc = [0, 1, 0] (1m Y offset in Body frame)
    Eigen::Vector3d l_ecc(0.0, 1.0, 0.0);
    
    // 4. Test Case 1: Perfect Match
    double t_query = 0.15; // Midpoint
    
    // Expected Pose at 0.15: [1.5, 0, 0]
    // Expected Axle Pos: [1.5, 0, 0] + R * l_ecc = [1.5, 0, 0] + I * [0, 1, 0] = [1.5, 1.0, 0]
    Eigen::Vector3d pos_meas_1(1.5, 1.0, 0.0);
    Eigen::Matrix3d weight = Eigen::Matrix3d::Identity();

    ContinuousGnssFactor factor1(t_query, dt_spline, control_points[0].timestamp(), pos_meas_1, weight);
    
    double residuals[3];
    double* params[5] = {
        control_points[0].pose_data(),
        control_points[1].pose_data(),
        control_points[2].pose_data(),
        control_points[3].pose_data(),
        l_ecc.data()
    };

    factor1(params[0], params[1], params[2], params[3], params[4], residuals);

    std::cout << "--- Case 1: Perfect Match ---" << std::endl;
    std::cout << "Residuals: " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
    
    if (std::abs(residuals[0]) < 1e-4 && std::abs(residuals[1]) < 1e-4 && std::abs(residuals[2]) < 1e-4) {
        std::cout << "PASS: Residuals near zero." << std::endl;
    } else {
        std::cout << "FAIL: Residuals too high." << std::endl;
        return 1;
    }

    // 5. Test Case 2: Mismatch
    // Measured at [1.5, 2.0, 0] (1m off in Y)
    Eigen::Vector3d pos_meas_2(1.5, 2.0, 0.0);
    ContinuousGnssFactor factor2(t_query, dt_spline, control_points[0].timestamp(), pos_meas_2, weight);

    factor2(params[0], params[1], params[2], params[3], params[4], residuals);
    
    std::cout << "\n--- Case 2: Mismatch (1m off in Y) ---" << std::endl;
    std::cout << "Residuals: " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;

    // Expected residual = P_axle - P_meas = [1.5, 1.0, 0] - [1.5, 2.0, 0] = [0, -1.0, 0]
    if (std::abs(residuals[0]) < 1e-4 && std::abs(residuals[1] + 1.0) < 1e-4 && std::abs(residuals[2]) < 1e-4) {
        std::cout << "PASS: Residuals match expected error." << std::endl;
    } else {
        std::cout << "FAIL: Residuals incorrect." << std::endl;
        return 1;
    }

    return 0;
}
