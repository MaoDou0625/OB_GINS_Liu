/*
 * Wheel-IMU Preintegration entry header
 * Mirrors src/preintegration/preintegration.h with independent classes/types
 */

#ifndef WHEEL_PREINTEGRATION_H
#define WHEEL_PREINTEGRATION_H

#include "preintegration_wheel_base.h"
#include "preintegration_wheel_normal.h"
#include "preintegration_wheel_odo.h"
#include "preintegration_wheel_earth.h"
#include "preintegration_wheel_earth_odo.h"

class WheelPreintegration {
public:
    using Options = int;

    enum PreintegrationOptions {
        PREINTEGRATION_NORMAL    = 0,
        PREINTEGRATION_ODO       = 1,
        PREINTEGRATION_EARTH     = 2,
        PREINTEGRATION_EARTH_ODO = 3,
    };

    static PreintegrationOptions getOptions(bool isuseodo, bool isearth) {
        int options = PREINTEGRATION_NORMAL;
        if (isuseodo) options += PREINTEGRATION_ODO;
        if (isearth) options += PREINTEGRATION_EARTH;
        return static_cast<PreintegrationOptions>(options);
    }

    static std::shared_ptr<WheelPreintegrationBase>
    createPreintegration(const std::shared_ptr<WheelIntegrationParameters> &parameters, const IMU &imu0,
                         const WheelIntegrationState &state, PreintegrationOptions options) {
        std::shared_ptr<WheelPreintegrationBase> preint;
        if (options == PREINTEGRATION_NORMAL) {
            preint = std::make_shared<WheelPreintegrationNormal>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_ODO) {
            preint = std::make_shared<WheelPreintegrationOdo>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH) {
            preint = std::make_shared<WheelPreintegrationEarth>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            preint = std::make_shared<WheelPreintegrationEarthOdo>(parameters, imu0, state);
        }
        return preint;
    }

    static int numPoseParameter() { return WheelPreintegrationBase::NUM_POSE; }

    static WheelIntegrationStateData stateToData(const WheelIntegrationState &state, PreintegrationOptions options) {
        if (options == PREINTEGRATION_NORMAL) {
            return WheelPreintegrationNormal::stateToData(state);
        } else if (options == PREINTEGRATION_ODO) {
            return WheelPreintegrationOdo::stateToData(state);
        } else if (options == PREINTEGRATION_EARTH) {
            return WheelPreintegrationEarth::stateToData(state);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            return WheelPreintegrationEarthOdo::stateToData(state);
        }
        return {};
    }

    static WheelIntegrationState stateFromData(const WheelIntegrationStateData &data, PreintegrationOptions options) {
        if (options == PREINTEGRATION_NORMAL) {
            return WheelPreintegrationNormal::stateFromData(data);
        } else if (options == PREINTEGRATION_ODO) {
            return WheelPreintegrationOdo::stateFromData(data);
        } else if (options == PREINTEGRATION_EARTH) {
            return WheelPreintegrationEarth::stateFromData(data);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            return WheelPreintegrationEarthOdo::stateFromData(data);
        }
        return {};
    }

    static int numMixParameter(PreintegrationOptions options) {
        if (options == PREINTEGRATION_NORMAL) {
            return WheelPreintegrationNormal::NUM_MIX;
        } else if (options == PREINTEGRATION_ODO) {
            return WheelPreintegrationOdo::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH) {
            return WheelPreintegrationEarth::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            return WheelPreintegrationEarthOdo::NUM_MIX;
        }
        return 0;
    }
};

#endif // WHEEL_PREINTEGRATION_H

