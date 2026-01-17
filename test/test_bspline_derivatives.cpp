#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include "src/spline/BSplineEvaluator.h"
#include "src/spline/ControlPoint.h"
#include <sophus/se3.hpp>

using namespace ob_gins::spline;

// Helper to convert angular velocity from body to world frame
Eigen::Vector3d body_to_world_angular_vel(const Sophus::SO3d& R_wb, const Eigen::Vector3d& w_body) {
    return R_wb * w_body;
}

// Helper to convert linear acceleration from body to world frame
Eigen::Vector3d body_to_world_linear_acc(const Sophus::SO3d& R_wb, const Eigen::Vector3d& a_body) {
    return R_wb * a_body;
}

TEST(BSplineDerivativesTest, NumericalVerification) {
    // Random seed for reproducibility
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.05); // Reduced noise for more stable spiral

    // Define the helical spiral parameters
    double radius = 10.0;
    double vz = 1.0; // Linear velocity in Z
    double omega_z = 0.5; // Angular velocity around Z
    double total_time = 10.0;
    double dt_cp = 0.1; // Control point interval
    int num_cps = static_cast<int>(total_time / dt_cp) + 4; // Add 3 for spline order + 1 extra for full trajectory

    std::vector<ControlPoint> control_points;

    // Generate control points for a helical spiral
    for (int i = 0; i < num_cps; ++i) {
        double t = i * dt_cp;
        double x = radius * std::cos(omega_z * t);
        double y = radius * std::sin(omega_z * t);
        double z = vz * t;

        // Position
        Eigen::Vector3d t_vec(x, y, z);

        // Orientation (rotating around Z-axis)
        Eigen::Quaterniond q_rot_z(Eigen::AngleAxisd(omega_z * t + noise(gen), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(noise(gen) * 0.1, Eigen::Vector3d::UnitY())); // Small pitch noise
        Eigen::Quaterniond q_roll(Eigen::AngleAxisd(noise(gen) * 0.1, Eigen::Vector3d::UnitX()));  // Small roll noise
        
        Sophus::SE3d pose(q_rot_z * q_pitch * q_roll, t_vec);
        control_points.emplace_back(t, pose);
    }
    
    // Define dt for BSplineEvaluator (matches control point interval)
    double dt_eval = dt_cp; 
    double delta_num_diff = 1e-5; // Small delta for numerical differentiation

    // Loop through the trajectory for verification
    // Avoid boundaries where spline evaluation might be less accurate or control points fewer
    for (double current_time = dt_eval * 3; current_time < total_time - dt_eval * 3; current_time += dt_eval * 0.5) {
        // Find the active segment's first control point index
        int segment_idx = static_cast<int>(current_time / dt_eval);
        
        // Normalize time 'u' within the current segment
        double u_normalized = (current_time - control_points[segment_idx].timestamp()) / dt_eval;

        // Ensure we have enough control points for the evaluation (spline order 3 needs 4 CPs)
        if (segment_idx + 3 >= control_points.size()) {
            std::cerr << "Not enough control points for time " << current_time << std::endl;
            break;
        }

        // A. Get Analytical Derivatives
        auto analytical_res = BSplineEvaluator::Evaluate(
            u_normalized, dt_eval, 
            control_points[segment_idx], control_points[segment_idx+1], 
            control_points[segment_idx+2], control_points[segment_idx+3]
        );

        // B. Compute Numerical Pose Derivatives (Velocity and Angular Velocity)
        double u_plus_dt_num = (current_time + delta_num_diff - control_points[segment_idx].timestamp()) / dt_eval;
        double u_minus_dt_num = (current_time - delta_num_diff - control_points[segment_idx].timestamp()) / dt_eval;

        auto res_plus = BSplineEvaluator::Evaluate(
            u_plus_dt_num, dt_eval, 
            control_points[segment_idx], control_points[segment_idx+1], 
            control_points[segment_idx+2], control_points[segment_idx+3]
        );
        auto res_minus = BSplineEvaluator::Evaluate(
            u_minus_dt_num, dt_eval, 
            control_points[segment_idx], control_points[segment_idx+1], 
            control_points[segment_idx+2], control_points[segment_idx+3]
        );
        
        // Numerical Linear Velocity (World Frame)
        Eigen::Vector3d v_world_num = (res_plus.pose.translation() - res_minus.pose.translation()) / (2 * delta_num_diff);
        
        // Numerical Angular Velocity (Body Frame)
        // w_body ≈ log(R(t-dt)^-1 * R(t+dt)) / (2 * delta_num_diff)
        Eigen::Vector3d w_body_num = (res_minus.pose.so3().inverse() * res_plus.pose.so3()).log() / (2 * delta_num_diff);

        // C. Assert Velocities
        EXPECT_NEAR(analytical_res.v_world.x(), v_world_num.x(), 1e-4);
        EXPECT_NEAR(analytical_res.v_world.y(), v_world_num.y(), 1e-4);
        EXPECT_NEAR(analytical_res.v_world.z(), v_world_num.z(), 1e-4);
        
        EXPECT_NEAR(analytical_res.w_body.x(), w_body_num.x(), 1e-4);
        EXPECT_NEAR(analytical_res.w_body.y(), w_body_num.y(), 1e-4);
        EXPECT_NEAR(analytical_res.w_body.z(), w_body_num.z(), 1e-4);


        // D. Compute Numerical Acceleration (Linear and Angular)
        Eigen::Vector3d a_world_num = (res_plus.v_world - res_minus.v_world) / (2 * delta_num_diff);
        Eigen::Vector3d alpha_body_num = (res_plus.w_body - res_minus.w_body) / (2 * delta_num_diff);


        // E. Assert Accelerations
        EXPECT_NEAR(analytical_res.a_world.x(), a_world_num.x(), 1e-4);
        EXPECT_NEAR(analytical_res.a_world.y(), a_world_num.y(), 1e-4);
        EXPECT_NEAR(analytical_res.a_world.z(), a_world_num.z(), 1e-4);
        
        EXPECT_NEAR(analytical_res.alpha_body.x(), alpha_body_num.x(), 1e-4);
        EXPECT_NEAR(analytical_res.alpha_body.y(), alpha_body_num.y(), 1e-4);
        EXPECT_NEAR(analytical_res.alpha_body.z(), alpha_body_num.z(), 1e-4);

        // F. IMU Consistency Check: a_world = R_wb * ( linear_accel_body + (w_body)^ v_body )
        Eigen::Matrix3d w_body_hat = Sophus::SO3d::hat(analytical_res.w_body);
        Eigen::Vector3d rhs_imu_check = analytical_res.pose.so3() * (analytical_res.linear_accel_body + w_body_hat * analytical_res.v_body);

        EXPECT_NEAR(analytical_res.a_world.x(), rhs_imu_check.x(), 1e-4);
        EXPECT_NEAR(analytical_res.a_world.y(), rhs_imu_check.y(), 1e-4);
        EXPECT_NEAR(analytical_res.a_world.z(), rhs_imu_check.z(), 1e-4);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}