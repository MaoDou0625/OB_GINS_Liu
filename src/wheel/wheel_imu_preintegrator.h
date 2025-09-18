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

#ifndef WHEEL_IMU_PREINTEGRATOR_H
#define WHEEL_IMU_PREINTEGRATOR_H

#include "wheel_imu_preintegration_base.h"
#include "wheel_imu_preintegration_earth.h"
#include "wheel_imu_preintegration_earth_odo.h"
#include "wheel_imu_preintegration_normal.h"
#include "wheel_imu_preintegration_odo.h"

class WheelImuPreintegrator {

public:
    WheelImuPreintegrator() = default;

    enum PreintegrationOptions {
        PREINTEGRATION_NORMAL    = 0,
        PREINTEGRATION_ODO       = 1,
        PREINTEGRATION_EARTH     = 2,
        PREINTEGRATION_EARTH_ODO = 3,
    };

    static PreintegrationOptions getOptions(bool isuseodo, bool isearth) {
        int options = PREINTEGRATION_NORMAL;

        if (isuseodo) {
            options += PREINTEGRATION_ODO;
        }
        if (isearth) {
            options += PREINTEGRATION_EARTH;
        }

        return static_cast<PreintegrationOptions>(options);
    }

    static std::shared_ptr<WheelPreintegrationBase>
        createPreintegration(const std::shared_ptr<IntegrationParameters> &parameters, const IMU &imu0,
                             const IntegrationState &state, PreintegrationOptions options) {
        std::shared_ptr<WheelPreintegrationBase> preintegration;

        if (options == PREINTEGRATION_NORMAL) {
            preintegration = std::make_shared<WheelPreintegrationNormal>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_ODO) {
            preintegration = std::make_shared<WheelPreintegrationOdo>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH) {
            preintegration = std::make_shared<WheelPreintegrationEarth>(parameters, imu0, state);
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            preintegration = std::make_shared<WheelPreintegrationEarthOdo>(parameters, imu0, state);
        }

        return preintegration;
    }

    static int numPoseParameter() {
        return WheelPreintegrationBase::NUM_POSE;
    }

    static IntegrationStateData stateToData(const IntegrationState &state, PreintegrationOptions options) {
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

    static IntegrationState stateFromData(const IntegrationStateData &data, PreintegrationOptions options) {
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
        int num = 0;
        if (options == PREINTEGRATION_NORMAL) {
            num = WheelPreintegrationNormal::NUM_MIX;
        } else if (options == PREINTEGRATION_ODO) {
            num = WheelPreintegrationOdo::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH) {
            num = WheelPreintegrationEarth::NUM_MIX;
        } else if (options == PREINTEGRATION_EARTH_ODO) {
            num = WheelPreintegrationEarthOdo::NUM_MIX;
        }
        return num;
    }
};

#endif // WHEEL_IMU_PREINTEGRATOR_H
