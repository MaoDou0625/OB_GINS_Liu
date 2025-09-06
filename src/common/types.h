/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct GNSS {
    double time;

    Vector3d blh;
    Vector3d std;
} GNSS;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

// Wheel-IMU raw rate sample (float64 rates: rad/s, m/s^2)
typedef struct WIMU {
    double time;      // seconds
    Eigen::Vector3d omega; // rad/s
    Eigen::Vector3d acc;   // m/s^2
} WIMU;

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

#endif // TYPES_H
