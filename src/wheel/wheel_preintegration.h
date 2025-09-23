/*
 * Wheel module: preintegration factory (independent entry, same behavior as main IMU).
 * Implements a wheel-specific factory that constructs the underlying preintegration
 * implementation directly, without calling the main IMU factory function. Behavior
 * remains identical (no additional extrinsic/lever handling at this stage).
 */
#ifndef WHEEL_PREINTEGRATION_H
#define WHEEL_PREINTEGRATION_H

#include "src/preintegration/preintegration_base.h"
#include "src/preintegration/preintegration_normal.h"
#include "src/preintegration/preintegration_odo.h"
#include "src/preintegration/preintegration_earth.h"
#include "src/preintegration/preintegration_earth_odo.h"
#include "src/preintegration/preintegration.h" // for Preintegration::PreintegrationOptions

namespace wheel {

struct WheelPreintegration {
    static std::shared_ptr<PreintegrationBase>
    Create(const std::shared_ptr<IntegrationParameters> &parameters,
           const IMU &imu0,
           const IntegrationState &state,
           Preintegration::PreintegrationOptions options) {
        switch (options) {
            case Preintegration::PREINTEGRATION_NORMAL:
                return std::make_shared<PreintegrationNormal>(parameters, imu0, state);
            case Preintegration::PREINTEGRATION_ODO:
                return std::make_shared<PreintegrationOdo>(parameters, imu0, state);
            case Preintegration::PREINTEGRATION_EARTH:
                return std::make_shared<PreintegrationEarth>(parameters, imu0, state);
            case Preintegration::PREINTEGRATION_EARTH_ODO:
                return std::make_shared<PreintegrationEarthOdo>(parameters, imu0, state);
            default:
                return std::make_shared<PreintegrationNormal>(parameters, imu0, state);
        }
    }
};

} // namespace wheel

#endif // WHEEL_PREINTEGRATION_H
