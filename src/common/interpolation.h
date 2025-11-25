#ifndef OB_GINS_INTERPOLATION_H
#define OB_GINS_INTERPOLATION_H

#include "src/common/types.h"

/**
 * @brief Checks if IMU interpolation is needed to align with a specific time point.
 * @param imu0 The IMU measurement before the time point.
 * @param imu1 The IMU measurement after the time point.
 * @param mid The target time point for alignment.
 * @return 
 * - 2: Interpolation is needed.
 * - 1: The time point is very close to imu1, so just use imu1.
 * - 0: The time point is not between imu0 and imu1.
 * - -1: The time point is very close to imu0, do nothing.
 */
int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);

/**
 * @brief Performs linear interpolation on an IMU measurement.
 * 
 * Given a measurement `imu01` that spans the time `mid`, this function splits it
 * into two new measurements: `imu00` (from start to `mid`) and `imu11` (from `mid` to end).
 * 
 * @param imu01 The original IMU measurement to be split.
 * @param imu00 [out] The resulting measurement for the first part of the interval.
 * @param imu11 [out] The resulting measurement for the second part of the interval.
 * @param mid The time point at which to split the measurement.
 */
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);


#endif // OB_GINS_INTERPOLATION_H
