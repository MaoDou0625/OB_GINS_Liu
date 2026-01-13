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

#include <sstream>
#include "src/common/angle.h"

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

#include "src/core/i_unified_preintegrator.h"
#include "src/wheel/preintegration_wheel_factor.h"
#include "src/wheel/imu_error_factor_wheel.h"

class WheelPreintegratorAdapter : public IUnifiedPreintegrator {
public:
    WheelPreintegratorAdapter(const std::shared_ptr<IntegrationParameters>& params, const IMU& first_imu,
                              const IntegrationState& init_state, WheelPreintegration::PreintegrationOptions options)
        : options_(options) {
        // Since the state structs are identical, we can safely reinterpret_cast.
        // This avoids a field-by-field copy. The alternative is a new constructor or a conversion function.
        auto wheel_params = std::reinterpret_pointer_cast<WheelIntegrationParameters>(params);
        const auto& wheel_init_state = reinterpret_cast<const WheelIntegrationState&>(init_state);
        preint_impl_ = WheelPreintegration::createPreintegration(wheel_params, first_imu, wheel_init_state, options);
    }

    void addNewImu(const IMU& imu) override {
        preint_impl_->addNewImu(imu);
    }

    double getEndTime() const override {
        return preint_impl_->endTime();
    }

    void propagateState(IntegrationState& state_to_update) const override {
        const auto& wheel_state = preint_impl_->currentState();
        // The structs are identical, so we can reinterpret_cast to copy.
        state_to_update = reinterpret_cast<const IntegrationState&>(wheel_state);
    }

    void syncStateFromData(const IntegrationStateData& data, IntegrationState& state_to_update) override {
        const auto& wheel_data = reinterpret_cast<const WheelIntegrationStateData&>(data);
        auto wheel_state = WheelPreintegration::stateFromData(wheel_data, options_);
        state_to_update = reinterpret_cast<const IntegrationState&>(wheel_state);
    }

    ceres::CostFunction* createImuFactor() override {
        return new WheelPreintegrationFactor(preint_impl_);
    }

    ceres::CostFunction* createImuErrorFactor() override {
        return new WheelImuErrorFactor(preint_impl_);
    }

    int getPoseParamSize() const override {
        return WheelPreintegration::numPoseParameter();
    }

    int getMixParamSize() const override {
        return WheelPreintegration::numMixParameter(options_);
    }

    std::string getDebugString() const override {
        if (!preint_impl_) return "Preint: null";
        const auto& delta = preint_impl_->deltaState();
        Vector3d att = Rotation::quaternion2euler(delta.q) * R2D;
        std::stringstream ss;
        ss << "dt=" << preint_impl_->deltaTime() 
           << " dP=[" << delta.p.transpose() << "]"
           << " dV=[" << delta.v.transpose() << "]"
           << " dAtt(deg)=[" << att.transpose() << "]";
        return ss.str();
    }

private:
    std::shared_ptr<WheelPreintegrationBase> preint_impl_;
    WheelPreintegration::PreintegrationOptions options_;
};


#endif // WHEEL_PREINTEGRATION_H

