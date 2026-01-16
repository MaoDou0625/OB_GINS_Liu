#ifndef OB_GINS_SPLINE_BSPLINE_EVALUATOR_H
#define OB_GINS_SPLINE_BSPLINE_EVALUATOR_H

#include "src/spline/ControlPoint.h"
#include <vector>
#include <iostream>

namespace ob_gins {
namespace spline {

class BSplineEvaluator {
public:
    struct Result {
        Sophus::SE3d pose;            // T_wb
        Eigen::Vector3d v_world;      // Linear velocity in World frame
        Eigen::Vector3d w_body;       // Angular velocity in Body frame
        Eigen::Vector3d a_world;      // Linear acceleration in World frame
        Eigen::Vector3d alpha_body;   // Angular acceleration in Body frame
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

    static Sophus::Matrix6d AdjointLieAlgebra(const Sophus::Vector6d& xi) {
        Sophus::Matrix6d ad_xi;
        ad_xi.setZero();
        
        Eigen::Vector3d rho = xi.head<3>(); // Translation part
        Eigen::Vector3d phi = xi.tail<3>(); // Rotation part

        Eigen::Matrix3d phi_hat = Sophus::SO3d::hat(phi);
        Eigen::Matrix3d rho_hat = Sophus::SO3d::hat(rho);

        ad_xi.block<3, 3>(0, 0) = phi_hat;
        ad_xi.block<3, 3>(0, 3) = rho_hat;
        ad_xi.block<3, 3>(3, 3) = phi_hat;

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
     * @brief Full Evaluation (Double precision) with Velocity and Acceleration
     */
    static Result Evaluate(double u, double dt, 
                           const ControlPoint& cp0, 
                           const ControlPoint& cp1, 
                           const ControlPoint& cp2, 
                           const ControlPoint& cp3) {
        
        // 1. Basis functions and derivatives
        Eigen::Vector4d U_vec;
        U_vec << 1, u, u * u, u * u * u;
        
        Eigen::Vector4d U_dot_vec;
        U_dot_vec << 0, 1, 2 * u, 3 * u * u;

        Eigen::Vector4d U_ddot_vec;
        U_ddot_vec << 0, 0, 2, 6 * u;

        Eigen::Matrix4d M = BaseMatrix();
        
        Eigen::Vector4d B = M.transpose() * U_vec;
        Eigen::Vector4d B_dot = M.transpose() * U_dot_vec;
        Eigen::Vector4d B_ddot = M.transpose() * U_ddot_vec;

        double b1 = B(1) + B(2) + B(3);
        double b2 = B(2) + B(3);
        double b3 = B(3);

        double b1_dot = B_dot(1) + B_dot(2) + B_dot(3);
        double b2_dot = B_dot(2) + B_dot(3);
        double b3_dot = B_dot(3);

        double b1_ddot = B_ddot(1) + B_ddot(2) + B_ddot(3);
        double b2_ddot = B_ddot(2) + B_ddot(3);
        double b3_ddot = B_ddot(3);

        // 2. Relative transformations
        Sophus::Vector6d Omega1 = (cp0.pose().inverse() * cp1.pose()).log();
        Sophus::Vector6d Omega2 = (cp1.pose().inverse() * cp2.pose()).log();
        Sophus::Vector6d Omega3 = (cp2.pose().inverse() * cp3.pose()).log();

        // 3. Pose
        Sophus::SE3d E1 = Sophus::SE3d::exp(Omega1 * b1);
        Sophus::SE3d E2 = Sophus::SE3d::exp(Omega2 * b2);
        Sophus::SE3d E3 = Sophus::SE3d::exp(Omega3 * b3);

        Sophus::SE3d T_wb = cp0.pose() * E1 * E2 * E3;

        // 4. Velocity
        Sophus::Vector6d term3 = Omega3 * b3_dot;
        Sophus::Vector6d term2 = Omega2 * b2_dot;
        Sophus::Vector6d term1 = Omega1 * b1_dot;

        Sophus::Matrix6d Adj_E3_inv = E3.inverse().Adj();
        Sophus::Matrix6d Adj_E2_inv = E2.inverse().Adj();

        Sophus::Vector6d xi_body = term3 + Adj_E3_inv * (term2 + Adj_E2_inv * term1);
        xi_body /= dt;

        Eigen::Vector3d v_body = xi_body.head<3>();
        Eigen::Vector3d w_body = xi_body.tail<3>();
        Eigen::Vector3d v_world = T_wb.so3() * v_body;

        // 5. Acceleration
        Sophus::Vector6d Z1 = Omega1 * b1_dot;
        Sophus::Vector6d Z2 = Omega2 * b2_dot;
        Sophus::Vector6d Z3 = Omega3 * b3_dot;

        Sophus::Vector6d Z1_dot = Omega1 * b1_ddot;
        Sophus::Vector6d Z2_dot = Omega2 * b2_ddot;
        Sophus::Vector6d Z3_dot = Omega3 * b3_ddot;

        Sophus::Matrix6d A2 = Adj_E3_inv;
        Sophus::Matrix6d A1 = A2 * Adj_E2_inv;

        Sophus::Matrix6d ad_neg_w_E3 = AdjointLieAlgebra(-Z3); 
        Sophus::Matrix6d A2_dot = A2 * ad_neg_w_E3;

        Sophus::Matrix6d ad_neg_w_E2 = AdjointLieAlgebra(-Z2);
        Sophus::Matrix6d A1_dot = A2_dot * Adj_E2_inv + A2 * (Adj_E2_inv * ad_neg_w_E2);

        Sophus::Vector6d xi_dot_u = Z3_dot 
                                  + A2 * Z2_dot + A2_dot * Z2 
                                  + A1 * Z1_dot + A1_dot * Z1;

        Sophus::Vector6d xi_accel = xi_dot_u / (dt * dt);

        Eigen::Vector3d a_body = xi_accel.head<3>();
        Eigen::Vector3d alpha_body = xi_accel.tail<3>();

        Eigen::Vector3d v_body_dot = a_body;
        Eigen::Vector3d w_cross_v = w_body.cross(v_body);
        
        Eigen::Vector3d a_world = T_wb.so3() * (v_body_dot + w_cross_v);

        Result res;
        res.pose = T_wb;
        res.v_world = v_world;
        res.w_body = w_body;
        res.a_world = a_world;
        res.alpha_body = alpha_body;

        return res;
    }
};

} // namespace spline
} // namespace ob_gins

#endif // OB_GINS_SPLINE_BSPLINE_EVALUATOR_H