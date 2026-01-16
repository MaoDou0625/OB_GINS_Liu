#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "src/spline/SplineInitializer.h"
#include "src/spline/BSplineEvaluator.h"

using namespace ob_gins::spline;

int main() {
    // 1. Generate Discrete Path (Circular)
    double radius = 10.0;
    double period = 10.0;
    double omega = 2.0 * M_PI / period;
    double t_start = 0.0;
    double t_end = 5.0;
    double dt_path = 0.01; // 100 Hz

    std::vector<std::pair<double, Sophus::SE3d>> path;
    for (double t = t_start; t <= t_end; t += dt_path) {
        double yaw = omega * t;
        Eigen::Vector3d pos(radius * std::sin(yaw), radius * (1.0 - std::cos(yaw)), 0.0);
        Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        path.push_back({t, Sophus::SE3d(Eigen::Quaterniond(R), pos)});
    }

    // 2. Initialize Spline
    double dt_spline = 0.1; // 10 Hz
    std::vector<ControlPoint> cps = SplineInitializer::InitializeFromPath(path, dt_spline);

    std::cout << "Generated " << cps.size() << " Control Points." << std::endl;

    // 3. Evaluate and Compare
    double error_sum = 0.0;
    int count = 0;

    double t_eval_start = cps[1].timestamp();
    double t_eval_end = cps[cps.size()-2].timestamp();

    for (double t = t_eval_start; t < t_eval_end; t += 0.05) { // Evaluate at 20Hz
        // Ground Truth
        double yaw = omega * t;
        Eigen::Vector3d pos_gt(radius * std::sin(yaw), radius * (1.0 - std::cos(yaw)), 0.0);
        
        // Spline Eval
        int idx = static_cast<int>(std::floor((t - cps[0].timestamp()) / dt_spline)) - 1;
        // Wait, index logic depends on start time.
        // t_start_cp = t_min - dt.
        // idx 0 is at t_min - dt.
        // idx 1 is at t_min.
        // if t = t_min, (t - (t_min-dt))/dt = dt/dt = 1.
        // We want idx such that CP[idx] is the first of the 4 points.
        // Segment [t_i, t_{i+1}) uses CP_{i-1}, CP_i, CP_{i+1}, CP_{i+2}.
        // So valid range for CP0..3 is [t1, t2).
        // t1 = cps[1].time.
        // So idx should be such that cps[idx+1].time <= t < cps[idx+2].time.
        // t = cps[idx+1].time + u * dt.
        // u = (t - cps[idx+1].time) / dt.
        // idx+1 = floor(t / dt) - offset?
        
        // Simpler: find idx such that cps[idx+1].time <= t
        int found_idx = -1;
        for(size_t i=0; i<cps.size()-3; ++i) {
            if (t >= cps[i+1].timestamp() && t < cps[i+2].timestamp()) {
                found_idx = i;
                break;
            }
        }
        
        if (found_idx == -1) continue;

        double u = (t - cps[found_idx+1].timestamp()) / dt_spline;
        
        auto res = BSplineEvaluator::Evaluate(u, dt_spline, cps[found_idx], cps[found_idx+1], cps[found_idx+2], cps[found_idx+3]);
        
        double err = (res.pose.translation() - pos_gt).norm();
        error_sum += err;
        count++;
    }

    double rmse = count > 0 ? (error_sum / count) : 0.0;
    std::cout << "RMSE of Initialization: " << rmse << " meters." << std::endl;

    // Simple interpolation usually gives error proportional to dt^2?
    // 10Hz spline on 10m radius circle.
    // Chord error...
    // Should be reasonably small (< 0.1m).
    
    if (rmse < 0.1) {
        std::cout << "SUCCESS: Initialization is close to ground truth." << std::endl;
    } else {
        std::cout << "FAIL: Initialization error too high." << std::endl;
        exit(1);
    }

    return 0;
}
