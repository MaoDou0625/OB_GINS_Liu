#include "interpolation.h"

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    if (imu0.time < mid && imu1.time > mid) {
        double dt = mid - imu0.time;

        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // close to the second epoch
        dt = imu1.time - mid;
        if (dt < 0.0001) {
            return 1;
        }

        return 2;
    }

    return 0;
}

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);
    imu00.is_wheel = buff.is_wheel;

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
    imu11.is_wheel = buff.is_wheel;
}
