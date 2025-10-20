/*
 * Wheel-IMU Preintegration: Independent integration state definitions
 * Mirrored from src/preintegration/integration_state.h with renamed types.
 */

#ifndef WHEEL_INTEGRATION_DEFINE_H
#define WHEEL_INTEGRATION_DEFINE_H

#include <Eigen/Geometry>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;

typedef struct WheelIntegrationState {
    double time;

    Vector3d p{0, 0, 0};
    Quaterniond q{1, 0, 0, 0};
    Vector3d v{0, 0, 0};

    Vector3d bg{0, 0, 0};
    Vector3d ba{0, 0, 0};

    Vector3d s{0, 0, 0};
    double sodo{0};
    Vector2d abv{0, 0};

    Vector3d sg{0, 0, 0};
    Vector3d sa{0, 0, 0};
} WheelIntegrationState;

typedef struct WheelIntegrationStateData {
    double time;

    double pose[7]; // pose : 3 + 4 = 7

    // mix parameters
    // vel + bias : 3 + 6 = 9
    // vel + bias + sodo : 3 + 6 + 1 = 10
    // vel + bias + sodo + abv : 3 + 6 + 1 + 2 = 12
    // vel + bias + scale : 3 + 6 + 6 = 15
    // vel + bias + sodo + scale : 3 + 6 + 1 + 6 = 16
    // vel + bias + sodo + scale + abv : 3 + 6 + 1 + 6 + 2 = 18
    double mix[18];
} WheelIntegrationStateData;

typedef struct WheelIntegrationParameters {
    // IMU噪声为白噪声, 积分为随机游走 VRW/ARW
    // 零偏和比例因子建模为一阶高斯马尔科夫过程：标准差及相关时间

    double acc_vrw;       // m / s^1.5
    double gyr_arw;       // rad / s^0.5
    double gyr_bias_std;  // rad / s
    double acc_bias_std;  // m / s^2
    double gyr_scale_std; // gyro scale std
    double acc_scale_std; // accel scale std
    double corr_time;     // s

    double gravity; // m / s^2

    Vector3d odo_std; // m/s
    double odo_srw;   // PPM / sqrt(Hz)

    Vector3d abv;  // rad
    Vector3d lodo; // m

    Vector3d station; // reference
} WheelIntegrationParameters;

#endif // WHEEL_INTEGRATION_DEFINE_H

