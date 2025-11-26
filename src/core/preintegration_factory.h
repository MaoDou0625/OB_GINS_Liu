#ifndef OB_GINS_PREINTEGRATION_FACTORY_H
#define OB_GINS_PREINTEGRATION_FACTORY_H

#include "src/core/i_unified_preintegrator.h"
#include "src/preintegration/preintegration.h"
#include "src/wheel/preintegration_wheel.h"
#include <functional>
#include <map>
#include <string>
#include <stdexcept>

/**
 * @struct PreintegrationCreationParams
 * @brief A struct to bundle all parameters required for creating a preintegrator.
 * 
 * This simplifies the factory's creator function signature.
 */
struct PreintegrationCreationParams {
    const std::shared_ptr<IntegrationParameters>& params;
    const IMU& first_imu;
    const IntegrationState& init_state;
    int options; // A generic int for options, to be cast by the concrete creator.
};


/**
 * @class PreintegrationFactory
 * @brief A singleton, registration-based factory for creating IUnifiedPreintegrator instances.
 * 
 * This factory allows for easy extension with new IMU types. To add a new type,
 * create a new class implementing IUnifiedPreintegrator and register its creation
 * logic here.
 */
class PreintegrationFactory {
public:
    using Creator = std::function<std::unique_ptr<IUnifiedPreintegrator>(const PreintegrationCreationParams&)>;

    /// Get the singleton instance of the factory.
    static PreintegrationFactory& getInstance() {
        static PreintegrationFactory instance;
        return instance;
    }

    /// Register a new preintegrator type with its creation function.
    void registerCreator(const std::string& type, Creator creator) {
        creators_[type] = creator;
    }

    /// Create a new preintegrator instance of the specified type.
    std::unique_ptr<IUnifiedPreintegrator> create(const std::string& type, const PreintegrationCreationParams& p) {
        if (creators_.find(type) == creators_.end()) {
            throw std::runtime_error("Unknown preintegrator type requested: " + type);
        }
        return creators_.at(type)(p);
    }

    /// Converts a canonical IntegrationState to its corresponding data array representation based on type.
    IntegrationStateData stateToData(const std::string& type, const IntegrationState& s, int options) {
        if (type == "standard") {
            return Preintegration::stateToData(s, static_cast<Preintegration::PreintegrationOptions>(options));
        } else if (type == "wheel") {
            // Since the structs are memory-layout-compatible, we can use reinterpret_cast
            const auto& wheel_s = reinterpret_cast<const WheelIntegrationState&>(s);
            auto wheel_data = WheelPreintegration::stateToData(wheel_s, static_cast<WheelPreintegration::PreintegrationOptions>(options));
            return reinterpret_cast<const IntegrationStateData&>(wheel_data);
        }
        throw std::runtime_error("Unknown type for stateToData: " + type);
    }

    /// Converts a data array representation back to a canonical IntegrationState based on type.
    IntegrationState stateFromData(const std::string& type, const IntegrationStateData& d, int options) {
        if (type == "standard") {
            return Preintegration::stateFromData(d, static_cast<Preintegration::PreintegrationOptions>(options));
        } else if (type == "wheel") {
            // Since the structs are memory-layout-compatible, we can use reinterpret_cast
            const auto& wheel_d = reinterpret_cast<const WheelIntegrationStateData&>(d);
            auto wheel_state = WheelPreintegration::stateFromData(wheel_d, static_cast<WheelPreintegration::PreintegrationOptions>(options));
            return reinterpret_cast<const IntegrationState&>(wheel_state);
        }
        throw std::runtime_error("Unknown type for stateFromData: " + type);
    }

private:
    PreintegrationFactory() = default;
    ~PreintegrationFactory() = default;
    PreintegrationFactory(const PreintegrationFactory&) = delete;
    PreintegrationFactory& operator=(const PreintegrationFactory&) = delete;

    std::map<std::string, Creator> creators_;
};

/**
 * @brief Initializes the factory by registering all known preintegrator types.
 * 
 * This function should be called once at application startup.
 */
inline void initializePreintegrationFactory() {
    auto& factory = PreintegrationFactory::getInstance();

    factory.registerCreator("standard", 
        [](const PreintegrationCreationParams& p) {
            return std::make_unique<StandardPreintegratorAdapter>(
                p.params, p.first_imu, p.init_state, 
                static_cast<Preintegration::PreintegrationOptions>(p.options)
            );
        }
    );

    factory.registerCreator("wheel", 
        [](const PreintegrationCreationParams& p) {
            return std::make_unique<WheelPreintegratorAdapter>(
                p.params, p.first_imu, p.init_state, 
                static_cast<WheelPreintegration::PreintegrationOptions>(p.options)
            );
        }
    );
}

#endif // OB_GINS_PREINTEGRATION_FACTORY_H
