#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>

#include "src/factors/BiasRandomWalkFactor.h"

using namespace ob_gins::factors;

int main() {
    // 1. Setup
    double dt = 0.1;
    double sigma = 0.01;
    
    Eigen::Vector3d b1(0.0, 0.0, 0.0);
    Eigen::Vector3d b2(1.0, 1.0, 1.0); // Initial guess far away

    // 2. Build Problem
    ceres::Problem problem;
    problem.AddParameterBlock(b1.data(), 3);
    problem.SetParameterBlockConstant(b1.data());
    problem.AddParameterBlock(b2.data(), 3);

    ceres::CostFunction* factor = new BiasRandomWalkFactor(dt, sigma);
    problem.AddResidualBlock(factor, nullptr, b1.data(), b2.data());

    // 3. Check Jacobians
    // Using simple numeric check manually or trust Ceres GradientChecker if I set it up.
    // Let's just solve and check convergence.
    // If Jacobians are wrong, it might converge slowly or fail, but this is a linear problem (quadratic cost).
    // It should converge in 1 iteration if Jacobians are correct (Newton step).
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << summary.BriefReport() << std::endl;

    std::cout << "Optimized b2: " << b2.transpose() << std::endl;

    if (b2.norm() < 1e-6) {
        std::cout << "SUCCESS: Bias converged to fixed neighbor." << std::endl;
    } else {
        std::cout << "FAIL: Bias did not converge." << std::endl;
        exit(1);
    }

    return 0;
}
