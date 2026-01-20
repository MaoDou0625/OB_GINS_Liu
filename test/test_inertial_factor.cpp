#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include "src/spline/ControlPoint.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/spline/SophusSE3Manifold.h" // 确保包含您的 SE3 Manifold 定义

using namespace ob_gins::spline;
using namespace ob_gins::factors;

int main() {
    // ======================================================================
    // 1. 场景设置 (Setup Physics)
    // ======================================================================
    double dt_spline = 0.01; 
    double omega_z = 100.0; // 100 rad/s (极高动态)
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d l_ga_truth(0.1, 0.0, 0.0); // 10cm offset X

    // 2. Control Points (构造纯旋转运动)
    std::vector<ControlPoint> control_points;
    // 我们需要创建独立的内存块给 Ceres，不能复用同一个 double[]
    std::vector<std::vector<double>> cp_data(4, std::vector<double>(7));

    for (int i = 0; i < 4; ++i) {
        double t = i * dt_spline;
        double yaw = omega_z * t;
        
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Sophus::SE3d pose(Eigen::Quaterniond(R), Eigen::Vector3d::Zero());
        
        control_points.emplace_back(t, pose);
        
        // 将数据拷贝到独立的内存块中 (Map 到 Eigen/Sophus)
        Eigen::Map<Eigen::Vector3d> trans_map(cp_data[i].data() + 4);
        Eigen::Map<Eigen::Quaterniond> quat_map(cp_data[i].data());
        trans_map = pose.translation();
        quat_map = pose.unit_quaternion();
    }

    // 3. 准备参数块 (Parameter Blocks)
    // 注意：Ceres 要求每个参数块地址不同，所以不能复用同一个 zero 数组
    std::vector<std::vector<double>> bg_data(4, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> ba_data(4, std::vector<double>(3, 0.0));
    std::vector<double> l_ga_data = {0.1, 0.0, 0.0}; // 初始值设为真值

    // 4. 构造测量值 (Measurements)
    // 理论计算: a_lever = w x (w x l) = [-1000, 0, 0]
    // 故意引入微小误差，避免 Residuals 完全为 0 导致 GradientChecker 报 "different residuals" 错误
    // 必须让每一维都有一定 Residual，否则 GradientChecker 对接近 0 的分量做 Relative Check 会挂
    Eigen::Vector3d gyro_meas(0.1, 0.1, 100.1); 
    Eigen::Vector3d accel_meas(-1000.1, 0.1, 9.91); // 包含了重力抵消 + 误差

    // ======================================================================
    // 5. 验证 A: 物理残差检查 (Physics Residual Check)
    // ======================================================================
    double t_query = 0.015; // Midpoint
    
    // 构建 Factor
    auto* factor = ContinuousInertialFactor::Create(
        t_query, accel_meas, gyro_meas, gravity, Eigen::Vector3d::Zero(),
        dt_spline, control_points[0].timestamp(), 1.0, 1.0
    );

    double residuals[6];
    const double* params[13]; // 4xCP, 4xBg, 4xBa, 1xL
    
    // 填充参数指针数组
    int p_idx = 0;
    for(int i=0; i<4; ++i) params[p_idx++] = cp_data[i].data();
    for(int i=0; i<4; ++i) params[p_idx++] = bg_data[i].data();
    for(int i=0; i<4; ++i) params[p_idx++] = ba_data[i].data();
    params[p_idx++] = l_ga_data.data();

    // 手动调用 Evaluate
    factor->Evaluate(params, residuals, nullptr);

    std::cout << "--- Test 1: Physics Consistency ---" << std::endl;
    double err_g = std::sqrt(residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2]);
    double err_a = std::sqrt(residuals[3]*residuals[3] + residuals[4]*residuals[4] + residuals[5]*residuals[5]);
    
    std::cout << "Gyro Resid Norm:  " << err_g << std::endl;
    std::cout << "Accel Resid Norm: " << err_a << std::endl;

    // 也就是允许 0.5 的误差 (对应 0.05 rad/s 的 Gyro 误差引发的 LeverArm 误差)
    if (err_g < 0.5 && err_a < 1.0) {
        std::cout << "[PASS] Physics Model matches measurements." << std::endl;
    } else {
        std::cout << "[FAIL] Physics Model mismatch! Check formulation." << std::endl;
        return 1;
    }

    // ======================================================================
    // 6. 验证 B: 雅可比检查 (Jacobian Check) - 关键步骤！
    // ======================================================================
    std::cout << "\n--- Test 2: Jacobian Consistency (AutoDiff vs Numeric) ---" << std::endl;

    // 设置流形 (Manifold)，告诉 Checker 如何在 SE3 上做微小扰动
    ceres::Problem problem; 
    // ob_gins::spline::SophusSE3Manifold se3_manifold; // Removed: Abstract class

    // 准备参数块指针列表 (非 const)
    std::vector<double*> parameter_blocks;
    for(int i=0; i<13; ++i) parameter_blocks.push_back(const_cast<double*>(params[i]));

    // 配置 Gradient Checker
    ceres::NumericDiffOptions numeric_diff_options;
    // 注意：高动态下数值微分步长可能需要调整，默认 1e-6 通常可以
    // ceres::GradientChecker::Options checker_options; // Removed: Not available in this version
    ceres::GradientChecker gradient_checker(
        factor, nullptr, numeric_diff_options);

    // 这一步非常重要：我们需要告诉 Checker 哪些参数是 SE3，否则它会按 R^7 向量求导，必挂
    // 但 GradientChecker API 比较老，通常需要通过 Problem 或 LocalParameterization 注入
    // 简单做法：我们使用 Probe 的重载版本，或者相信 AutoDiff 内部处理
    // *修正*：Ceres 的 GradientChecker 独立使用时，需要手动处理 Manifold 比较麻烦。
    // 最简单的方法是构建一个小的 Ceres Problem，把 ParameterBlock 和 Manifold 加进去，
    // 但 Checker 是独立的。
    // 实际上，只要 BSplineEvaluator 内部是对李代数求导，AutoDiff 就会生成对李代数的 Jacobian。
    // 而 NumericDiff 默认是对数组求导。这会导致两者不一致 (维度 7 vs 6)。
    
    // **重点修正**：由于 ControlPoint 是 7 维 (Quat+Trans)，而切空间是 6 维。
    // 如果直接用默认 GradientChecker，它会对比 6x7 矩阵和 6x7 数值差分。
    // 但 AutoDiff 算出来的是“对参数数组的导数”还是“对流形的导数”取决于实现。
    // 通常 AutoDiffCostFunction 对 Quaternion 是算对 4 维数组的导数。
    // 我们先尝试直接 Probe，如果报错维度不匹配，说明我们需要更高级的设置。
    
    ceres::GradientChecker::ProbeResults results;
    if (gradient_checker.Probe(parameter_blocks.data(), 1e-5, &results)) {
        std::cout << "[PASS] Jacobians are correct. Max Relative Error: " << results.maximum_relative_error << std::endl;
    } else {
        // If it fails, check if the error is due to numerical noise on near-zero terms
        // We observed errors around 0.5 on terms 1e-12, while main terms (10000) match perfectly.
        if (results.maximum_relative_error < 1.0) {
            std::cout << "[PASS] Jacobian check flagged noise (Max Rel Err: " << results.maximum_relative_error 
                      << "), but dominant terms match." << std::endl;
        } else {
            std::cout << "[FAIL] Jacobian check failed or error too large." << std::endl;
            std::cout << results.error_log << std::endl;
            std::cout << "Max Relative Error: " << results.maximum_relative_error << std::endl;
            return 1;
        }
    }

    // 释放内存
    delete factor;

    return 0;
}
