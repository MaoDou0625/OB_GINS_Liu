#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "src/spline/ControlPoint.h"
#include "src/factors/ContinuousInertialFactor.h"

using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main() {
    // 1. Setup
    double dt_spline = 0.01; // Higher resolution for high dynamics
    double omega_z = 100.0; // 100 rad/s
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d l_ga(0.1, 0.0, 0.0); // 10cm offset X

    // 2. Control Points (Pure Rotation around Z)
    std::vector<ControlPoint> control_points;
    for (int i = 0; i < 4; ++i) {
        double t = i * dt_spline;
        double yaw = omega_z * t;
        
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Sophus::SE3d pose(Eigen::Quaterniond(R), Eigen::Vector3d::Zero());
        
        control_points.emplace_back(t, pose);
    }

    // Biases (Zero)
    Eigen::Vector3d bg(0,0,0);
    Eigen::Vector3d ba(0,0,0);

    // 3. Define Measurements
    // Gyro: [0, 0, 100]
    Eigen::Vector3d gyro_meas(0.0, 0.0, 100.0);

    // Accel:
    // a_kinematic (Gyro Center) = 0.
    // a_lever = w x (w x l) = [0,0,100] x ([0,0,100] x [0.1,0,0])
    //         = [0,0,100] x [0, 10, 0]
    //         = [-1000, 0, 0]
    // a_meas = a_kinematic + a_lever - g + bias
    //        = 0 + [-1000, 0, 0] - [0,0,-9.81] + 0
    //        = [-1000, 0, 9.81]
    Eigen::Vector3d accel_meas(-1000.0, 0.0, 9.81);

    // 4. Evaluate Factor
    double t_query = 0.015; // Midpoint
    
    // Updated Constructor (no l_ga)
    ContinuousInertialFactor factor(t_query, accel_meas, gyro_meas, gravity, 
                                    dt_spline, control_points[0].timestamp());
    
    double residuals[6];
    
    // Create Dummy Bias Pointers
    double bg_arr[3] = {0,0,0};
    double ba_arr[3] = {0,0,0};
    double l_ga_arr[3] = {0.1, 0.0, 0.0}; // Pass l_ga here
    
    factor(control_points[0].pose_data(),
           control_points[1].pose_data(),
           control_points[2].pose_data(),
           control_points[3].pose_data(),
           bg_arr, bg_arr, bg_arr, bg_arr,
           ba_arr, ba_arr, ba_arr, ba_arr,
           l_ga_arr, // New Param
           residuals);

    std::cout << "Residuals:" << std::endl;
    std::cout << "Gyro: " << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << std::endl;
    std::cout << "Accel: " << residuals[3] << ", " << residuals[4] << ", " << residuals[5] << std::endl;

    // Check
    double err_g = std::sqrt(residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2]);
    double err_a = std::sqrt(residuals[3]*residuals[3] + residuals[4]*residuals[4] + residuals[5]*residuals[5]);

    std::cout << "Error Gyro: " << err_g << std::endl;
    std::cout << "Error Accel: " << err_a << std::endl;

    if (err_g < 1e-2 && err_a < 1e-2) {
        std::cout << "SUCCESS: Factor correctly modeled high dynamic lever arm effect." << std::endl;
    } else {
        std::cout << "FAIL: Residuals too high." << std::endl;
        exit(1);
    }

    return 0;
}
