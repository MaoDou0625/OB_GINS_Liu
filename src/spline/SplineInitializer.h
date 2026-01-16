#ifndef OB_GINS_SPLINE_SPLINE_INITIALIZER_H
#define OB_GINS_SPLINE_SPLINE_INITIALIZER_H

#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>

#include "src/spline/ControlPoint.h"

namespace ob_gins {
namespace spline {

class SplineInitializer {
public:
    static std::vector<ControlPoint> InitializeFromPath(
        const std::vector<std::pair<double, Sophus::SE3d>>& path, 
        double dt) {
        
        if (path.empty()) return {};

        double t_min = path.front().first;
        double t_max = path.back().first;

        // Ensure padding to cover the full range
        // For Cubic B-Spline with Uniform Knots:
        // Segment [t_i, t_{i+1}) depends on CP_{i-1}, CP_i, CP_{i+1}, CP_{i+2}.
        // So to validly evaluate at t_min, we need CPs starting before t_min.
        // Let's create CPs starting at t_min - dt.
        // And ending at t_max + dt (at least).
        
        // Start time for first CP
        // Align to dt if desired? Or just offset from t_min.
        // Let's just offset.
        double t_start_cp = t_min - dt; 
        
        std::vector<ControlPoint> cps;
        
        // Generate CPs until we pass t_max + padding
        // We need coverage up to t_max.
        // Valid evaluation up to t requires CP at t + dt? 
        // Let's extend well beyond t_max.
        
        int num_cp = static_cast<int>(std::ceil((t_max - t_min) / dt)) + 5; 
        // +1 for ceil round up, +1 for start offset, +3 for end padding support?
        // Let's just loop.

        for (int i = 0; i < num_cp; ++i) {
            double t = t_start_cp + i * dt;
            
            // Interpolate from path
            Sophus::SE3d pose = InterpolatePath(path, t);
            
            cps.emplace_back(t, pose);
        }

        return cps;
    }

private:
    static Sophus::SE3d InterpolatePath(const std::vector<std::pair<double, Sophus::SE3d>>& path, double t) {
        if (t <= path.front().first) return path.front().second;
        if (t >= path.back().first) return path.back().second;

        // Binary search for lower bound
        auto it = std::lower_bound(path.begin(), path.end(), t, 
            [](const std::pair<double, Sophus::SE3d>& p, double val) {
                return p.first < val;
            });
        
        // it points to first element >= t.
        // so it is the "next" element.
        // prev is it - 1.
        if (it == path.begin()) return it->second; // Should be handled by t <= check but safe.

        const auto& p_next = *it;
        const auto& p_prev = *(it - 1);

        double t1 = p_prev.first;
        double t2 = p_next.first;
        double alpha = (t - t1) / (t2 - t1);

        // Linear interpolation on manifold
        // P(alpha) = P1 * exp(log(P1^-1 * P2) * alpha)
        Sophus::SE3d P1 = p_prev.second;
        Sophus::SE3d P2 = p_next.second;
        
        Sophus::Vector6d delta = (P1.inverse() * P2).log();
        Sophus::SE3d P_interp = P1 * Sophus::SE3d::exp(delta * alpha);
        
        return P_interp;
    }
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_SPLINE_INITIALIZER_H
