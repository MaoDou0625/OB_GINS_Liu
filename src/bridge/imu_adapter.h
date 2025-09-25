// IMU adapter: route between main IMU and wheel IMU implementations.
// Keep the main flow simple: decide by IMU::is_wheel and dispatch here.

#pragma once

#include <memory>
#include <deque>

#include "src/common/types.h"

// Main IMU preintegration and factor
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"
#include "src/preintegration/imu_error_factor.h"

// Wheel IMU interfaces
#include "src/wheel/integration_state_wheel.h"
#include "src/wheel/preintegration_wheel.h"
#include "src/wheel/preintegration_wheel_factor.h"
#include "src/wheel/imu_error_factor_wheel.h"

namespace Adapter {

class UnifiedPreintegrator {
public:
    using Options = Preintegration::PreintegrationOptions;

    static std::shared_ptr<UnifiedPreintegrator>
    Create(const std::shared_ptr<IntegrationParameters> &params, const IMU &first_imu,
           const IntegrationState &init_state, Options opt, bool is_wheel);

    void addNewImu(const IMU &imu);

    double endTime() const;

    IntegrationState      currentStateMain() const;
    WheelIntegrationState currentStateWheel() const;

    std::shared_ptr<PreintegrationBase>      rawMain() const { return preint_main_; }
    std::shared_ptr<WheelPreintegrationBase> rawWheel() const { return preint_wheel_; }

    bool isWheel() const { return is_wheel_; }

private:
    explicit UnifiedPreintegrator(bool is_wheel) : is_wheel_(is_wheel) {}

    bool is_wheel_{false};
    std::shared_ptr<PreintegrationBase>      preint_main_;
    std::shared_ptr<WheelPreintegrationBase> preint_wheel_;
};

inline UnifiedPreintegrator::Options GetOptions(bool isuseodo, bool isearth) {
    return Preintegration::getOptions(isuseodo, isearth);
}

inline int NumPoseParameter() { return Preintegration::numPoseParameter(); }

inline int NumMixParameter(UnifiedPreintegrator::Options options) {
    return Preintegration::numMixParameter(options);
}

inline IntegrationStateData StateToData(const IntegrationState &s, UnifiedPreintegrator::Options opt) {
    return Preintegration::stateToData(s, opt);
}

inline IntegrationState StateFromData(const IntegrationStateData &d, UnifiedPreintegrator::Options opt) {
    return Preintegration::stateFromData(d, opt);
}

inline ceres::CostFunction *MakePreintFactor(const std::shared_ptr<UnifiedPreintegrator> &up) {
    if (!up) return nullptr;
    if (up->isWheel() && up->rawWheel()) return new WheelPreintegrationFactor(up->rawWheel());
    if (up->rawMain()) return new PreintegrationFactor(up->rawMain());
    return nullptr;
}

inline ceres::CostFunction *MakeImuErrorFactor(const std::shared_ptr<UnifiedPreintegrator> &up) {
    if (!up) return nullptr;
    if (up->isWheel() && up->rawWheel()) return new WheelImuErrorFactor(up->rawWheel());
    if (up->rawMain()) return new ImuErrorFactor(up->rawMain());
    return nullptr;
}

} // namespace Adapter

