#ifndef OB_GINS_I_UNIFIED_PREINTEGRATOR_H
#define OB_GINS_I_UNIFIED_PREINTEGRATOR_H

#include "src/common/types.h"
#include "src/preintegration/integration_state.h"

#include <ceres/ceres.h>
#include <memory>

/**
 * @class IUnifiedPreintegrator
 * @brief An abstract interface for a preintegration module.
 *
 * This class defines a polymorphic interface for different types of IMU
 * preintegrators (e.g., standard, wheel, visual). It allows the ImuChain
 * to handle various preintegrator types without type-specific conditional logic.
 * The implementing classes are responsible for managing the concrete preintegration
 * logic and handling conversions between their specific state types and the
 * canonical IntegrationState used by the ImuChain.
 */
class IUnifiedPreintegrator {
public:
    virtual ~IUnifiedPreintegrator() = default;

    /// Add a new IMU measurement to the integration.
    virtual void addNewImu(const IMU& imu) = 0;

    /// Get the end time of the current pre-integration segment.
    virtual double getEndTime() const = 0;

    /// Propagate the internal concrete state (e.g., WheelIntegrationState)
    /// into the canonical IntegrationState object used by ImuChain.
    virtual void propagateState(IntegrationState& state_to_update) const = 0;

    /// Synchronize the internal concrete state from the canonical IntegrationStateData
    /// object that has been updated by the optimizer.
    virtual void syncStateFromData(const IntegrationStateData& data, IntegrationState& state_to_update) = 0;

    /// Create the main IMU pre-integration factor for a Ceres problem.
    virtual ceres::CostFunction* createImuFactor() = 0;

    /// Create the IMU bias/error factor for a Ceres problem.
    virtual ceres::CostFunction* createImuErrorFactor() = 0;

    /// Get the size of the pose parameter block for this preintegrator type.
    virtual int getPoseParamSize() const = 0;

    /// Get the size of the mixed (speed/bias/etc.) parameter block.
    virtual int getMixParamSize() const = 0;
};

#endif // OB_GINS_I_UNIFIED_PREINTEGRATOR_H
