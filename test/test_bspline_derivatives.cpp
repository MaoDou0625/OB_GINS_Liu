#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include "src/spline/BSplineEvaluator.h"
#include "src/spline/ControlPoint.h"
#include <sophus/se3.hpp>

using namespace ob_gins::spline;

// Simple assertion helper
void assert_near(const std::string& name, const Eigen::Vector3d& val1, const Eigen::Vector3d& val2, double tol = 1e-4) {
    double err = (val1 - val2).norm();
    std::cout << std::left << std::setw(20) << name 
              << " | Error: " << std::setw(12) << err 
              << (err < tol ? " [PASS]" : " [FAIL]") << std::endl;
    
    if (err >= tol) {
        std::cout << "  Expected: " << val1.transpose() << std::endl;
        std::cout << "  Actual:   " << val2.transpose() << std::endl;
        exit(1);
    }
}

int main() {
    // Random seed for reproducibility
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    double dt_spline = 0.1;
    std::vector<ControlPoint> control_points;
    
    std::cout << "Initializing Control Points..." << std::endl;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d t_vec(i * 1.0 + noise(gen), noise(gen), noise(gen));
        Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
        Sophus::SE3d pose(q, t_vec);
        control_points.emplace_back(i * dt_spline, pose);
    }

    double t_query = 0.15;
    
    // Normalized time u for t_query relative to the segment starting at t=0.1
    // t0=0.0, t1=0.1, t2=0.2, t3=0.3
    // Active segment is [t1, t2) -> [0.1, 0.2)
    // u = (t - t1) / dt = (0.15 - 0.1) / 0.1 = 0.5
    double u = (t_query - control_points[1].timestamp()) / dt_spline;

    // Analytical Evaluation
    BSplineEvaluator::Result analytical = BSplineEvaluator::Evaluate(
        u, dt_spline, 
        control_points[0], control_points[1], control_points[2], control_points[3]
    );

    // Numerical Differentiation Parameters
    double eps = 1e-5;
    double dt_num = eps * dt_spline; // Small time step for finite difference

    // Calculate Pose at t - dt_num and t + dt_num
    double u_prev = (t_query - dt_num - control_points[1].timestamp()) / dt_spline;
    double u_next = (t_query + dt_num - control_points[1].timestamp()) / dt_spline;

    auto res_prev = BSplineEvaluator::Evaluate(u_prev, dt_spline, control_points[0], control_points[1], control_points[2], control_points[3]);
    auto res_next = BSplineEvaluator::Evaluate(u_next, dt_spline, control_points[0], control_points[1], control_points[2], control_points[3]);

    // Numerical Linear Velocity (World)
    Eigen::Vector3d v_world_num = (res_next.pose.translation() - res_prev.pose.translation()) / (2 * dt_num);

    // Numerical Angular Velocity (Body)
    // w_body ≈ log(R(t-dt)^-1 * R(t+dt)) / (2*dt)
    Sophus::SO3d R_prev = res_prev.pose.so3();
    Sophus::SO3d R_next = res_next.pose.so3();
    Eigen::Vector3d w_body_num = (R_prev.inverse() * R_next).log() / (2 * dt_num);

    std::cout << "\n--- Velocity Check ---" << std::endl;
    assert_near("V_world", analytical.v_world, v_world_num);
    assert_near("W_body", analytical.w_body, w_body_num);

    // Numerical Acceleration
    // Differentiating analytical velocities from neighbors
    Eigen::Vector3d v_world_prev = res_prev.v_world;
    Eigen::Vector3d v_world_next = res_next.v_world;
    Eigen::Vector3d a_world_num = (v_world_next - v_world_prev) / (2 * dt_num);

    Eigen::Vector3d w_body_prev = res_prev.w_body;
    Eigen::Vector3d w_body_next = res_next.w_body;
    Eigen::Vector3d alpha_body_num = (w_body_next - w_body_prev) / (2 * dt_num);

    std::cout << "\n--- Acceleration Check ---" << std::endl;
    assert_near("A_world", analytical.a_world, a_world_num);
    assert_near("Alpha_body", analytical.alpha_body, alpha_body_num);

    std::cout << "\nAll checks passed successfully!" << std::endl;
    return 0;
}