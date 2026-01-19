#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include "src/spline/BSplineEvaluator.h"
#include "src/spline/SplineInitializer.h"
#include "src/factors/ContinuousInertialFactor.h"
#include "src/factors/ContinuousGnssFactor.h"
#include "src/factors/pose_manifold.h" // Changed from SophusSE3Manifold.h

#include <ceres/autodiff_manifold.h>

using namespace ob_gins;
using namespace ob_gins::spline;
using namespace ob_gins::factors;

// 辅助：生成螺旋线真值
Sophus::SE3d get_ground_truth(double t, double R=20.0, double vz=1.0, double omega=0.5) {
    Eigen::Vector3d pos(R * std::cos(omega * t), R * std::sin(omega * t), vz * t);
    // Z轴旋转 + 稍微倾斜一点以激活更多轴的动态
    Eigen::Matrix3d Rot = Eigen::AngleAxisd(omega * t + M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                        * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return Sophus::SE3d(Eigen::Quaterniond(Rot), pos);
}

// Manifold for Sophus::SE3 using AutoDiff
struct SophusSE3Functor {
    template <typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
        Eigen::Map<const Sophus::SE3<T>> T_x(x);
        Eigen::Map<const Sophus::Vector6<T>> d(delta);
        Eigen::Map<Sophus::SE3<T>> T_out(x_plus_delta);
        // Right multiplication: T_new = T * exp(delta)
        T_out = T_x * Sophus::SE3<T>::exp(d);
        return true;
    }

    template <typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const {
        Eigen::Map<const Sophus::SE3<T>> T_y(y);
        Eigen::Map<const Sophus::SE3<T>> T_x(x);
        Eigen::Map<Sophus::Vector6<T>> d(y_minus_x);
        // Tangent delta = log(T_x^-1 * T_y)
        d = (T_x.inverse() * T_y).log();
        return true;
    }
};

int main() {
    // 1. 配置参数
    double total_time = 5.0;
    double dt_spline = 0.3; // 控制点间隔 (较稀疏，故意制造初始化误差)
    double dt_imu = 0.01;   // 100Hz IMU
    double dt_gnss = 1.0;   // 1Hz GNSS
    Eigen::Vector3d gravity(0, 0, -9.81);

    std::cout << "=== 1. Generating Synthetic Data ===" << std::endl;
    // 生成用于初始化的路径 (模拟里程计)
    std::vector<std::pair<double, Sophus::SE3d>> odom_path;
    for(double t=0; t<=total_time; t+=dt_spline) {
        odom_path.push_back({t, get_ground_truth(t)});
    }

    // 2. 初始化样条 (含 Corner Cutting 误差)
    std::cout << "=== 2. Initializing Spline (Expect Error) ===" << std::endl;
    std::vector<ControlPoint> cps = SplineInitializer::InitializeFromPath(odom_path, dt_spline);
    
    // 记录优化前的误差
    double error_before = 0.0;
    int count_before = 0;
    for(double t=1.0; t<4.0; t+=0.1) {
        // 简易求值，不处理边界
        int k = std::floor((t - cps[0].timestamp()) / dt_spline);
        if(k-1 < 0 || k+2 >= cps.size()) continue;
        double u = (t - cps[k].timestamp()) / dt_spline;
        
        // Correct usage: Evaluate(u, dt, cp0, cp1, cp2, cp3)
        // Interval [tk, tk+1] uses P_{k-1}, P_k, P_{k+1}, P_{k+2}
        auto res = BSplineEvaluator::Evaluate(u, dt_spline, 
            cps[k-1], cps[k], cps[k+1], cps[k+2]);
            
        error_before += (res.pose.translation() - get_ground_truth(t).translation()).norm();
        count_before++;
    }
    std::cout << "Average Pos Error BEFORE Optimization: " << (count_before > 0 ? error_before / count_before : 0.0) << " m (Should be > 0.1m)" << std::endl;

    // 3. 构建优化问题
    std::cout << "=== 3. Building Factor Graph ===" << std::endl;
    ceres::Problem problem;
    ceres::Manifold* se3_manifold = new ceres::AutoDiffManifold<SophusSE3Functor, 7, 6>;


    // 添加参数块
    for(auto& cp : cps) {
        // Correct usage: pose_data() instead of data()
        problem.AddParameterBlock(cp.pose_data(), 7, se3_manifold);
        problem.AddParameterBlock(cp.bg_data(), 3);
        problem.AddParameterBlock(cp.ba_data(), 3);
    }
    
    // 偏差参数 (假设为0且固定，为了简化测试核心的轨迹优化)
    // double bg[3] = {0,0,0}; // Removed
    // double ba[3] = {0,0,0}; // Removed
    double l_ga[3] = {0,0,0}; // 无杆臂
    problem.AddParameterBlock(l_ga, 3);
    problem.SetParameterBlockConstant(l_ga); // Fix lever arm for now

    // A. 添加 IMU 因子
    for (double t = cps[2].timestamp(); t < cps[cps.size()-2].timestamp(); t += dt_imu) {
        Sophus::SE3d pose = get_ground_truth(t);
        double R = 20.0, w = 0.5;
        Eigen::Vector3d a_world(-R*w*w*std::cos(w*t), -R*w*w*std::sin(w*t), 0);
        Eigen::Vector3d a_meas = pose.so3().inverse() * (a_world - gravity);
        Eigen::Vector3d w_meas = pose.so3().inverse() * Eigen::Vector3d(0, 0, w); 

        // 寻找关联的控制点索引
        int idx = std::floor((t - cps[0].timestamp()) / dt_spline) - 1;
        if(idx < 0 || idx + 3 >= cps.size()) continue;

        auto* cost = ContinuousInertialFactor::Create(
            t, a_meas, w_meas, gravity, dt_spline, cps[idx].timestamp() 
        );
        
        // Correct usage: pose_data()
        problem.AddResidualBlock(cost, nullptr, 
            cps[idx].pose_data(), cps[idx+1].pose_data(), cps[idx+2].pose_data(), cps[idx+3].pose_data(),
            cps[idx].bg_data(), cps[idx+1].bg_data(), cps[idx+2].bg_data(), cps[idx+3].bg_data(),
            cps[idx].ba_data(), cps[idx+1].ba_data(), cps[idx+2].ba_data(), cps[idx+3].ba_data(),
            l_ga
        );
    }

    // B. 添加 GNSS 因子 (固定轨迹位置)
    for (double t = cps[2].timestamp(); t < cps[cps.size()-2].timestamp(); t += dt_gnss) {
        Sophus::SE3d gt = get_ground_truth(t);
        
        // 寻找索引
        int idx = std::floor((t - cps[0].timestamp()) / dt_spline) - 1;
        if(idx < 0 || idx + 3 >= cps.size()) continue;

        auto* gnss_cost = ContinuousGnssFactor::Create(
            t, dt_spline, cps[idx].timestamp(), gt.translation(), Eigen::Matrix3d::Identity()
        );
        problem.AddResidualBlock(gnss_cost, nullptr,
            cps[idx].pose_data(), cps[idx+1].pose_data(), cps[idx+2].pose_data(), cps[idx+3].pose_data(),
            l_ga
        );
    }

    // 4. 求解
    std::cout << "=== 4. Solving... ===" << std::endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // 5. 验证结果
    double error_after = 0.0;
    int count_after = 0;
    for(double t=1.0; t<4.0; t+=0.1) {
        int k = std::floor((t - cps[0].timestamp()) / dt_spline);
        if(k-1 < 0 || k+2 >= cps.size()) continue;
        double u = (t - cps[k].timestamp()) / dt_spline;
        
        // Correct usage: Evaluate(u, dt, cp0, cp1, cp2, cp3)
        // Interval [tk, tk+1] uses P_{k-1}, P_k, P_{k+1}, P_{k+2}
        auto res = BSplineEvaluator::Evaluate(u, dt_spline, 
            cps[k-1], cps[k], cps[k+1], cps[k+2]);
            
        error_after += (res.pose.translation() - get_ground_truth(t).translation()).norm();
        count_after++;
    }
    double avg_err = (count_after > 0 ? error_after / count_after : 0.0);
    std::cout << "Average Pos Error AFTER Optimization:  " << avg_err << " m" << std::endl;

    if (avg_err < 0.05) {
        std::cout << "SUCCESS: Optimization successfully corrected the initialization error!" << std::endl;
        return 0;
    } else {
        std::cout << "FAIL: Optimization did not converge to ground truth." << std::endl;
        return 1;
    }
}
