#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "src/spline/BSplineEvaluator.h"
#include "src/spline/ControlPoint.h"
#include <sophus/se3.hpp>

using namespace ob_gins::spline;

// 统计辅助类
struct ErrorStats {
    double max_error = 0.0;
    double sum_sq_error = 0.0;
    int count = 0;

    void update(double error) {
        max_error = std::max(max_error, error);
        sum_sq_error += error * error;
        count++;
    }

    double rmse() const {
        return (count > 0) ? std::sqrt(sum_sq_error / count) : 0.0;
    }
};

TEST(BSplineDerivativesTest, NumericalVerificationWithReport) {
    // 1. 准备螺旋线轨迹数据
    std::vector<Eigen::VectorXd> cps_data; // 存储原始数据用于传给 Evaluate
    double radius = 10.0;
    double vz = 5.0;
    double omega = 1.0; // rad/s
    double total_time = 5.0;
    double dt_cp = 0.1; // 控制点间隔 10Hz
    int num_cps = static_cast<int>(total_time / dt_cp);

    // 生成控制点 (简单的螺旋运动)
    for (int i = 0; i < num_cps; ++i) {
        double t = i * dt_cp;
        // 构造 SE3: 绕Z轴旋转 + 沿Z轴上升
        Sophus::SE3d pose(
            Sophus::SO3d::exp(Eigen::Vector3d(0, 0, omega * t)),
            Eigen::Vector3d(radius * std::cos(omega * t), radius * std::sin(omega * t), vz * t)
        );
        // 将 SE3 转换为数据数组 (7维: quat_x, y, z, w, tx, ty, tz)
        Eigen::VectorXd cp_vec(7);
        cp_vec.head<4>() = pose.so3().unit_quaternion().coeffs();
        cp_vec.tail<3>() = pose.translation();
        cps_data.push_back(cp_vec);
    }
    
    // 2. 验证参数
    double delta = 1e-5; // 数值差分步长
    ErrorStats err_v_world, err_w_body, err_a_world, err_alpha_body;

    std::cout << "\n================ BSpline Derivative Verification Report ================\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Time(u) |  V_err   |  W_err   |  Acc_err | Alpha_err | Status\n";
    std::cout << "--------+----------+----------+----------+-----------+--------\n";

    // 3. 循环验证 (避开边界)
    // 我们需要 4 个控制点来计算一个区间，所以从索引 0 开始取 4 个点
    // t 对应的是 cps[1] 和 cps[2] 之间的时间
    for (int i = 0; i < num_cps - 4; ++i) {
        // 当前处理的 4 个控制点
        const double* cp0 = cps_data[i].data();
        const double* cp1 = cps_data[i+1].data();
        const double* cp2 = cps_data[i+2].data();
        const double* cp3 = cps_data[i+3].data();

        // Helper to map raw pointer to SE3
        auto map_se3 = [](const double* ptr) {
            // Assuming layout: qx, qy, qz, qw, tx, ty, tz
            Eigen::Map<const Eigen::Quaterniond> q(ptr);
            Eigen::Map<const Eigen::Vector3d> t(ptr + 4);
            return Sophus::SE3d(q, t);
        };

        // 在 [0, 1] 归一化时间内采样
        for (double u = 0.1; u < 0.95; u += 0.2) {
            
            // --- A. 解析解 (Analytical) ---
            // Note: Evaluate takes (u, dt, T0, T1, T2, T3)
            auto res = BSplineEvaluator::Evaluate(u, dt_cp, map_se3(cp0), map_se3(cp1), map_se3(cp2), map_se3(cp3));

            // --- B. 数值解 (Numerical Central Difference) ---
            // t + delta
            auto res_p = BSplineEvaluator::Evaluate(u + delta/dt_cp, dt_cp, map_se3(cp0), map_se3(cp1), map_se3(cp2), map_se3(cp3));
            // t - delta
            auto res_m = BSplineEvaluator::Evaluate(u - delta/dt_cp, dt_cp, map_se3(cp0), map_se3(cp1), map_se3(cp2), map_se3(cp3));

            // 1. 线速度验证: p_world 的导数
            Eigen::Vector3d v_world_num = (res_p.pose.translation() - res_m.pose.translation()) / (2 * delta);
            double err_v = (res.v_world - v_world_num).norm();

            // 2. 角速度验证: R_wb 的导数 (Body系)
            // w_body = (R^T * R_dot) -> log((R_{t-d}^-1 * R_{t+d}) / 2d)
            Eigen::Vector3d w_body_num = (res_m.pose.so3().inverse() * res_p.pose.so3()).log() / (2 * delta);
            double err_w = (res.w_body - w_body_num).norm();

            // 3. 线加速度验证: v_world 的导数 [CRITICAL]
            Eigen::Vector3d a_world_num = (res_p.v_world - res_m.v_world) / (2 * delta);
            double err_a = (res.a_world - a_world_num).norm();

            // 4. 角加速度验证: w_body 的导数
            Eigen::Vector3d alpha_body_num = (res_p.w_body - res_m.w_body) / (2 * delta);
            double err_alpha = (res.alpha_body - alpha_body_num).norm();

            // 统计
            err_v_world.update(err_v);
            err_w_body.update(err_w);
            err_a_world.update(err_a);
            err_alpha_body.update(err_alpha);

            // 打印部分采样点
            if (i % 5 == 0 && std::abs(u - 0.5) < 0.1) {
                std::cout << "Seg " << std::setw(2) << i << "  | " 
                          << err_v << " | " << err_w << " | " << err_a << " | " << err_alpha 
                          << " | " << ((err_a < 1e-3) ? "OK" : "FAIL") << "\n";
            }

            // 断言 (允许一定数值误差，加速度通常误差稍大)
            EXPECT_NEAR(err_v, 0.0, 1e-4);
            EXPECT_NEAR(err_w, 0.0, 1e-4);
            EXPECT_NEAR(err_a, 0.0, 1e-3); 
            EXPECT_NEAR(err_alpha, 0.0, 1e-3);
        }
    }

    std::cout << "--------+----------+----------+----------+-----------+--------\n";
    std::cout << "RMSE    | " << err_v_world.rmse() << " | " << err_w_body.rmse() 
              << " | " << err_a_world.rmse() << " | " << err_alpha_body.rmse() << "\n";
    std::cout << "Max Err | " << err_v_world.max_error << " | " << err_w_body.max_error 
              << " | " << err_a_world.max_error << " | " << err_alpha_body.max_error << "\n";
    std::cout << "========================================================================\n";
}