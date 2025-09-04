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

#ifndef ODOFILELOADER_H
#define ODOFILELOADER_H

#include "fileloader.h"
#include "src/common/types.h"

// Simple odometer file loader.
// Expected formats:
//  - 2 columns: time, v
//  - 3 columns: time, v_left, v_right (averaged)
// Values are assumed to be linear speed in m/s (vehicle frame, +X forward).
class OdoFileLoader : public FileLoader {

public:
    OdoFileLoader() = delete;
    explicit OdoFileLoader(const string &filename, int columns = 2) {
        open(filename, columns, FileLoader::TEXT);
    }

    const ODO &next() {
        data_ = load();

        odo_.time = data_[0];
        if (columns_ == 2) {
            odo_.vel = data_[1];
        } else if (columns_ >= 3) {
            // Average of two wheels if provided, otherwise use the second column
            odo_.vel = 0.5 * (data_[1] + data_[2]);
        }

        return odo_;
    }

private:
    ODO odo_{};
    vector<double> data_;
};

#endif // ODOFILELOADER_H

