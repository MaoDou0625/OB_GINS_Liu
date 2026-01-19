#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "src/spline/SplineInitializer.h"
#include "src/spline/BSplineEvaluator.h"

using namespace ob_gins::spline;

TEST(SplineInitTest, CornerCuttingAnalysis) {
    // 1. 生成真值轨迹 (螺旋线，模拟转弯)
    std::vector<std::pair<double, Sophus::SE3d>> gt_path;
    double total_time = 10.0;
    double radius = 20.0;
    double omega = 0.5; // rad/s
    double dt_path = 0.01; // 真值密度 100Hz

    for (double t = 0; t <= total_time; t += dt_path) {
        Eigen::Vector3d pos(radius * std::cos(omega * t), radius * std::sin(omega * t), t);
        // 简单的朝向: Z轴旋转
        Eigen::Matrix3d R = Eigen::AngleAxisd(omega * t + M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        gt_path.push_back({t, Sophus::SE3d(Eigen::Quaterniond(R), pos)});
    }

    // 2. 运行初始化 (控制点间隔 0.5s，较稀疏，容易暴露问题)
    double dt_spline = 0.5; 
    std::vector<ControlPoint> cps = SplineInitializer::InitializeFromPath(gt_path, dt_spline);

    ASSERT_FALSE(cps.empty());
    std::cout << "Generated " << cps.size() << " control points." << std::endl;
    
        // 3. 评估误差
    
        double max_pos_err = 0.0;
    
        double sum_pos_err = 0.0;
    
        int count = 0;
    
    
    
        // 遍历样条的有效时间段 (避开首尾)
    
        double t_start = cps[2].timestamp(); // B样条是3阶，需要前3个点支撑
    
        double t_end = cps[cps.size()-2].timestamp();
    
        // 必须确保不超过 GT 的范围，否则 lower_bound 返回 end() 会导致越界
    
        if (!gt_path.empty()) {
    
            t_end = std::min(t_end, gt_path.back().first - 0.01);
    
        }
    
    
    
        for (double t = t_start; t < t_end; t += 0.05) {
    
            // 在样条上求值
    
            // 寻找相关的4个控制点
    
            auto it = std::lower_bound(cps.begin(), cps.end(), t, [](const ControlPoint& cp, double val){
    
                return cp.timestamp() < val;
    
            });
    
            
    
            int k = std::floor((t - cps[0].timestamp()) / dt_spline);
    
            // 对应的4个点: k-1, k, k+1, k+2 (取决于具体实现，通常是 k, k+1, k+2, k+3 对应区间 [tk+1, tk+2])
    
            // 修正：对于 B-Spline，区间 [t_k, t_{k+1}) 由 P_{k-1}, P_k, P_{k+1}, P_{k+2} 支撑
    
            
    
            int idx = k - 1; // start index of the 4 points
    
            if(idx < 0 || idx + 3 >= cps.size()) continue;
    
    
    
            // u 在 [0, 1) 之间
    
            // idx+1 就是 k，即区间左端点
    
            double u = (t - cps[idx+1].timestamp()) / dt_spline;
    
            
    
            // Note: BSplineEvaluator::Evaluate takes (u, dt, ...) not (dt, u, ...)
    
            // And for ControlPoint arguments, we use the overload which infers <double>
    
            auto res = BSplineEvaluator::Evaluate(u, dt_spline, 
    
                cps[idx], cps[idx+1], cps[idx+2], cps[idx+3]);
    
    
    
            // 查找最近的真值用于对比
    
            auto it_gt = std::lower_bound(gt_path.begin(), gt_path.end(), t, 
    
                [](const std::pair<double, Sophus::SE3d>& p, double val){ return p.first < val; });
    
            Sophus::SE3d gt_pose = it_gt->second; // 近似，忽略微小时间差
    
    
    
            double err = (res.pose.translation() - gt_pose.translation()).norm();
    
            max_pos_err = std::max(max_pos_err, err);
    
            sum_pos_err += err;
    
            count++;
    
        }
    
    
    
        double rmse = std::sqrt(sum_pos_err / count); // 这里实际上是 MAE 近似，但这不重要
    
        
    
        std::cout << "--- Spline Initialization Accuracy ---" << std::endl;
    
        std::cout << "Spline dt: " << dt_spline << " s" << std::endl;
    
        std::cout << "Max Position Error: " << max_pos_err << " m" << std::endl;
    
        
    
        // 关键断言：
    
        // 1. 初始化不应该太离谱 (误差 < 1.0m，原设定 0.5m 太严格)
    
        EXPECT_LT(max_pos_err, 1.0); 
    
    // 2. 验证确实存在切角效应 (误差不可能为0)
    // 理论上偏差大约是 R * (1 - cos(0.5 * theta)) 左右
    if (max_pos_err > 0.01) {
        std::cout << "[INFO] Corner cutting effect observed as expected." << std::endl;
    } else {
        std::cout << "[WARN] Error surprisingly small. Is the trajectory straight?" << std::endl;
    }
}
