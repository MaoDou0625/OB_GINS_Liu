/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 */

#ifndef GNSSFILELOADER_H
#define GNSSFILELOADER_H

#include "fileloader.h"
#include "src/common/angle.h"
#include "src/common/types.h"

class GnssFileLoader : public FileLoader {
public:
    GnssFileLoader() = delete;
    explicit GnssFileLoader(const string &filename, int columns = 7) {
        open(filename, columns, FileLoader::TEXT);
    }

    // Robust parse: expects at least time + 3 position fields.
    const GNSS &next() {
        data_ = load();

        if (data_.size() < 4) {
            return gnss_;
        }

        gnss_.time   = data_[0];
        gnss_.blh[0] = data_[1];
        gnss_.blh[1] = data_[2];
        gnss_.blh[2] = data_[3];

        const size_t n = data_.size();
        if (n >= 10) {
            memcpy(gnss_.std.data(), &data_[7], 3 * sizeof(double));
        } else if (n >= 7) {
            memcpy(gnss_.std.data(), &data_[4], 3 * sizeof(double));
        } else {
            gnss_.std.setOnes();
        }

        // Validate std to avoid division by zero in optimization
        for (int i = 0; i < 3; ++i) {
            if (gnss_.std[i] < 1e-4) gnss_.std[i] = 1.0;
        }

        gnss_.blh[0] *= D2R;
        gnss_.blh[1] *= D2R;
        return gnss_;
    }

private:
    GNSS gnss_{};
    vector<double> data_{};
};

#endif // GNSSFILELOADER_H

