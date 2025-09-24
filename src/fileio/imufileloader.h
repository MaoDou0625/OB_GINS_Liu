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

#ifndef IMUFILELOADER_H
#define IMUFILELOADER_H

#include "fileloader.h"
#include "src/common/types.h"

class ImuFileLoader : public FileLoader {

public:
    ImuFileLoader() = delete;
    ImuFileLoader(const string &filename, int columns, int rate = 200, bool is_wheel_source = false) {
        open(filename, columns, FileLoader::TEXT);

        dt_ = 1.0 / (double) rate;

        imu_.time = 0;
        mark_wheel_ = is_wheel_source;
    }

    const IMU &next() {
        imu_pre_ = imu_;

        data_ = load();
        // tag IMU source type for downstream use
        imu_.is_wheel = mark_wheel_;

        // Debug 安全读取：按实际解析列数进行拷贝，防止越界
        const size_t n = data_.size();
        if (n < 7) {
            // 列数不足，使用名义采样间隔推进时间并置零增量，避免崩溃
            imu_.time   = imu_pre_.time + dt_;
            imu_.dt     = dt_;
            imu_.dtheta.setZero();
            imu_.dvel.setZero();
            imu_.odovel = 0.0;
            return imu_;
        }

        imu_.time = data_[0];
        memcpy(imu_.dtheta.data(), &data_[1], 3 * sizeof(double));
        memcpy(imu_.dvel.data(), &data_[4], 3 * sizeof(double));

        double dt = imu_.time - imu_pre_.time;
        if (dt < 0.1) {
            imu_.dt = dt;
        } else {
            imu_.dt = dt_;
        }

        // 增量形式（按实际列数判断 8/9 列）
        if (n >= 9) {
            imu_.odovel = 0.5 * (data_[7] + data_[8]) * imu_.dt;
        } else if (n >= 8) {
            imu_.odovel = data_[7] * imu_.dt;
        } else {
            imu_.odovel = 0.0;
        }

        return imu_;
    }

private:
    double dt_;
    bool mark_wheel_{false};

    IMU imu_, imu_pre_;
    vector<double> data_;
};

#endif // IMUFILELOADER_H
