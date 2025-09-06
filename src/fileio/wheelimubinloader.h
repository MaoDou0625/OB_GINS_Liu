/*
 * Wheel-IMU binary loader: float64 Nx7 [t, gx, gy, gz, ax, ay, az]
 */

#ifndef WHEELIMUBINLOADER_H
#define WHEELIMUBINLOADER_H

#include "fileloader.h"
#include "src/common/types.h"

class WheelImuBinLoader : public FileLoader {
public:
    WheelImuBinLoader() = delete;
    explicit WheelImuBinLoader(const std::string &filename, int columns = 7) {
        open(filename, columns, FileLoader::BINARY);
    }

    const WIMU &next() {
        data_ = load();
        w_.time = data_[0];
        memcpy(w_.omega.data(), &data_[1], 3 * sizeof(double));
        memcpy(w_.acc.data(),   &data_[4], 3 * sizeof(double));
        return w_;
    }

private:
    WIMU w_{};
    std::vector<double> data_;
};

#endif // WHEELIMUBINLOADER_H

