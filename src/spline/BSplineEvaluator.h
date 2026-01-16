#ifndef OB_GINS_SPLINE_BSPLINE_EVALUATOR_H
#define OB_GINS_SPLINE_BSPLINE_EVALUATOR_H

#include "src/spline/ControlPoint.h"
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ob_gins {
namespace spline {

class BSplineEvaluator {
public:
    template <typename T>
    struct Result {
        Sophus::SE3<T> pose;            // T_wb
        Eigen::Matrix<T, 3, 1> v_world;      // Linear velocity in World frame
        Eigen::Matrix<T, 3, 1> v_body;       // Linear velocity in Body frame
        Eigen::Matrix<T, 3, 1> w_body;       // Angular velocity in Body frame
        Eigen::Matrix<T, 3, 1> a_world;      // Linear acceleration in World frame
        Eigen::Matrix<T, 3, 1> linear_accel_body; // Linear acceleration in Body frame (v_body_dot)
        Eigen::Matrix<T, 3, 1> alpha_body;   // Angular acceleration in Body frame
    };

    // Matrix for cubic B-spline
    static Eigen::Matrix4d BaseMatrix() {
        Eigen::Matrix4d M;
        M << 1,  4,  1, 0,
            -3,  0,  3, 0,
             3, -6,  3, 0,
            -1,  3, -3, 1;
        M /= 6.0;
        return M;
    }

    template <typename T>
    static Sophus::Matrix6<T> AdjointLieAlgebra(const Sophus::Vector6<T>& xi) {
        Sophus::Matrix6<T> ad_xi;
        ad_xi.setZero();
        
        Eigen::Matrix<T, 3, 1> rho = xi.template head<3>(); // Translation part
        Eigen::Matrix<T, 3, 1> phi = xi.template tail<3>(); // Rotation part

        Eigen::Matrix<T, 3, 3> phi_hat = Sophus::SO3<T>::hat(phi);
        Eigen::Matrix<T, 3, 3> rho_hat = Sophus::SO3<T>::hat(rho);

        ad_xi.template block<3, 3>(0, 0) = phi_hat;
        ad_xi.template block<3, 3>(0, 3) = rho_hat;
        ad_xi.template block<3, 3>(3, 3) = phi_hat;

        return ad_xi;
    }

    /**
     * @brief Templated Pose Evaluation for Ceres AutoDiff
     */
    template <typename T>
    static Sophus::SE3<T> EvaluatePose(T u, 
                                       const Sophus::SE3<T>& T0,
                                       const Sophus::SE3<T>& T1,
                                       const Sophus::SE3<T>& T2,
                                       const Sophus::SE3<T>& T3) {
        
        Eigen::Matrix<T, 4, 1> U_vec;
        U_vec << T(1), u, u * u, u * u * u;
        
        // Cast BaseMatrix to T
        Eigen::Matrix<T, 4, 4> M = BaseMatrix().cast<T>();
        Eigen::Matrix<T, 4, 1> B = M.transpose() * U_vec;

        T b1 = B(1) + B(2) + B(3);
        T b2 = B(2) + B(3);
        T b3 = B(3);

        Sophus::Vector6<T> Omega1 = (T0.inverse() * T1).log();
        Sophus::Vector6<T> Omega2 = (T1.inverse() * T2).log();
        Sophus::Vector6<T> Omega3 = (T2.inverse() * T3).log();

        Sophus::SE3<T> E1 = Sophus::SE3<T>::exp(Omega1 * b1);
        Sophus::SE3<T> E2 = Sophus::SE3<T>::exp(Omega2 * b2);
        Sophus::SE3<T> E3 = Sophus::SE3<T>::exp(Omega3 * b3);

        return T0 * E1 * E2 * E3;
    }

    /**
     * @brief Full Evaluation (Templated) with Velocity and Acceleration
     */
    template <typename T>
    static Result<T> Evaluate(T u, T dt, 
                           const Sophus::SE3<T>& T0, 
                           const Sophus::SE3<T>& T1, 
                           const Sophus::SE3<T>& T2, 
                           const Sophus::SE3<T>& T3) {
        
        // 1. Basis functions and derivatives
        Eigen::Matrix<T, 4, 1> U_vec;
        U_vec << T(1), u, u * u, u * u * u;
        
        Eigen::Matrix<T, 4, 1> U_dot_vec;
        U_dot_vec << T(0), T(1), T(2) * u, T(3) * u * u;

        Eigen::Matrix<T, 4, 1> U_ddot_vec;
        U_ddot_vec << T(0), T(0), T(2), T(6) * u;

        Eigen::Matrix<T, 4, 4> M = BaseMatrix().cast<T>();
        
        Eigen::Matrix<T, 4, 1> B = M.transpose() * U_vec;
        Eigen::Matrix<T, 4, 1> B_dot = M.transpose() * U_dot_vec;
        Eigen::Matrix<T, 4, 1> B_ddot = M.transpose() * U_ddot_vec;

        T b1 = B(1) + B(2) + B(3);
        T b2 = B(2) + B(3);
        T b3 = B(3);

        T b1_dot = B_dot(1) + B_dot(2) + B_dot(3);
        T b2_dot = B_dot(2) + B_dot(3);
        T b3_dot = B_dot(3);

        T b1_ddot = B_ddot(1) + B_ddot(2) + B_ddot(3);
        T b2_ddot = B_ddot(2) + B_ddot(3);
        T b3_ddot = B_ddot(3);

        // 2. Relative transformations
        Sophus::Vector6<T> Omega1 = (T0.inverse() * T1).log();
        Sophus::Vector6<T> Omega2 = (T1.inverse() * T2).log();
        Sophus::Vector6<T> Omega3 = (T2.inverse() * T3).log();

        // 3. Pose
        Sophus::SE3<T> E1 = Sophus::SE3<T>::exp(Omega1 * b1);
        Sophus::SE3<T> E2 = Sophus::SE3<T>::exp(Omega2 * b2);
        Sophus::SE3<T> E3 = Sophus::SE3<T>::exp(Omega3 * b3);

        Sophus::SE3<T> T_wb = T0 * E1 * E2 * E3;

        // 4. Velocity
        Sophus::Vector6<T> term3 = Omega3 * b3_dot;
        Sophus::Vector6<T> term2 = Omega2 * b2_dot;
        Sophus::Vector6<T> term1 = Omega1 * b1_dot;

        Sophus::Matrix6<T> Adj_E3_inv = E3.inverse().Adj();
        Sophus::Matrix6<T> Adj_E2_inv = E2.inverse().Adj();

        Sophus::Vector6<T> xi_body = term3 + Adj_E3_inv * (term2 + Adj_E2_inv * term1);
        xi_body /= dt;

        Eigen::Matrix<T, 3, 1> v_body = xi_body.template head<3>();
        Eigen::Matrix<T, 3, 1> w_body = xi_body.template tail<3>();
        Eigen::Matrix<T, 3, 1> v_world = T_wb.so3() * v_body;

        // 5. Acceleration
        Sophus::Vector6<T> Z1 = Omega1 * b1_dot;
        Sophus::Vector6<T> Z2 = Omega2 * b2_dot;
        Sophus::Vector6<T> Z3 = Omega3 * b3_dot;

        Sophus::Vector6<T> Z1_dot = Omega1 * b1_ddot;
        Sophus::Vector6<T> Z2_dot = Omega2 * b2_ddot;
        Sophus::Vector6<T> Z3_dot = Omega3 * b3_ddot;

        Sophus::Matrix6<T> A2 = Adj_E3_inv;
        // Sophus::Matrix6<T> A1 = A2 * Adj_E2_inv; // Unused variable? No, used in formula below (wait, formula below used A1_dot)
        // Let's re-derive or check existing code logic.
        // Original code: Sophus::Matrix6d A1 = A2 * Adj_E2_inv;
        // And then A1_dot uses A2 and A2_dot.
        // xi_dot_u formula: Z3_dot + A2*Z2_dot + A2_dot*Z2 + A1*Z1_dot + A1_dot*Z1
        
        Sophus::Matrix6<T> A1 = A2 * Adj_E2_inv;

        Sophus::Matrix6<T> ad_neg_w_E3 = AdjointLieAlgebra<T>(-Z3); 
        Sophus::Matrix6<T> A2_dot = A2 * ad_neg_w_E3;

        Sophus::Matrix6<T> ad_neg_w_E2 = AdjointLieAlgebra<T>(-Z2);
        Sophus::Matrix6<T> A1_dot = A2_dot * Adj_E2_inv + A2 * (Adj_E2_inv * ad_neg_w_E2);

        Sophus::Vector6<T> xi_dot_u = Z3_dot 
                                  + A2 * Z2_dot + A2_dot * Z2 
                                  + A1 * Z1_dot + A1_dot * Z1;

        Sophus::Vector6<T> xi_accel = xi_dot_u / (dt * dt);

        Eigen::Matrix<T, 3, 1> a_body = xi_accel.template head<3>();
        Eigen::Matrix<T, 3, 1> alpha_body = xi_accel.template tail<3>();

        Eigen::Matrix<T, 3, 1> v_body_dot = a_body;
        Eigen::Matrix<T, 3, 1> w_cross_v = w_body.cross(v_body);
        
        Eigen::Matrix<T, 3, 1> a_world = T_wb.so3() * (v_body_dot + w_cross_v);

        Result<T> res;
        res.pose = T_wb;
        res.v_world = v_world;
        res.v_body = v_body; // Assign new member
        res.w_body = w_body;
        res.a_world = a_world;
        res.linear_accel_body = a_body; // Assign new member (which is v_body_dot)
        res.alpha_body = alpha_body;

        return res;
    }

    // Helper overload for non-templated ControlPoints (compatibility)
    static Result<double> Evaluate(double u, double dt, 
                           const ControlPoint& cp0, 
                           const ControlPoint& cp1, 
                           const ControlPoint& cp2, 
                           const ControlPoint& cp3) {
        return Evaluate<double>(u, dt, cp0.pose(), cp1.pose(), cp2.pose(), cp3.pose());
    }
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_BSPLINE_EVALUATOR_H
