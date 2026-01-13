#ifndef OB_GINS_IMU_CHAIN_H
#define OB_GINS_IMU_CHAIN_H

#include "src/common/earth.h"
#include "src/common/types.h"
#include "src/fileio/filesaver.h"
#include "src/fileio/imufileloader.h"
#include "src/factors/gnss_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/core/i_unified_preintegrator.h"
#include "src/common/interpolation.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <memory>
#include <string>
#include <limits>
#include <vector>

// Forward declaration
class MarginalizationInfo;

/*
 * @class ImuChain
 * @brief Encapsulates all data and operations for a single IMU processing pipeline.
 *
 * This class manages the state, parameters, data I/O, and factor creation
 * for one IMU chain (e.g., a body-mounted IMU or a wheel-mounted IMU).
 */
class ImuChain {
public:
    // Constructor: Initializes the chain from configuration nodes.
    ImuChain(std::string name, const YAML::Node& chain_node, const YAML::Node& global_config);

    // Initializes the first state of the chain using the first GNSS measurement.
    bool initializeFirstState(const GNSS& initial_gnss, const Vector3d& station_origin);

    // Aligns the IMU data stream to a given start time, optionally performing static alignment.
    void alignAndSync(double start_time, const Vector3d& blh);

    // Processes IMU data up to a specified time boundary, including interpolation.
    void processImuUpTo(double time_boundary);
    
    // Updates the C++ state objects from the raw double arrays used by the optimizer.
    void syncStatesFromOptimizer();

    // Adds all relevant parameter blocks for this chain to the Ceres problem.
    void addParameterBlocksTo(ceres::Problem& problem, size_t max_idx);

    // Adds IMU preintegration factors for this chain to the Ceres problem.
    void addImuFactorsTo(ceres::Problem& problem, size_t max_idx);

    // Adds a GNSS factor for this chain at a specific time index.
    bool addGnssFactorTo(ceres::Problem& problem, const GNSS& gnss, ceres::LossFunction* loss,
                         const std::deque<double>& time_list, ceres::ResidualBlockId* out_id = nullptr);

    // Adds the final bias factor for this chain.
    void addBiasFactorTo(ceres::Problem& problem, size_t idx);

    // Adds the first IMU and GNSS (if applicable) factors to a marginalization info object.
    void addFactorsToMarginalizationInfo(std::shared_ptr<MarginalizationInfo>& marginalization_info, const GNSS& gnss);
    
    // Slides the window by removing the oldest state and preintegrator.
    void slideWindow(const std::deque<double>& time_list, const std::deque<GNSS>& gnss_list);

    // Writes the latest navigation result to file.
    void writeResult(double time, const Vector3d& origin, size_t state_idx = std::numeric_limits<size_t>::max());

    // Creates and adds a new preintegrator for the next segment.
    void startNewPreintegration();

    // Updates gravity parameter for this chain.
    void updateGravity(double gravity) { parameters_->gravity = gravity; }

    // Sliding window size helper.
    size_t windowSize() const { return state_list_.size() > 0 ? state_list_.size() - 1 : 0; }
    size_t segmentCount() const { return preintegrators_.size(); }

    // Accessors
    const std::string& getName() const { return name_; }
    const std::string& getType() const { return type_; }
    bool isEnabled() const { return is_enabled_; }
    double currentImuTime() const { return imu_cur_.time; }
    IMU& currentImu() { return imu_cur_; }
    const IMU& currentImu() const { return imu_cur_; }
    IMU& previousImu() { return imu_pre_; }
    const IMU& previousImu() const { return imu_pre_; }
    ImuFileLoader* getLoader() { return loader_.get(); }
    std::deque<std::unique_ptr<IUnifiedPreintegrator>>& getPreintegrators() { return preintegrators_; }
    std::vector<IntegrationState>& getStateList() { return state_list_; }
    std::vector<IntegrationStateData>& getStateDataList() { return state_data_list_; }


private:
    // Helper to safely read a Vector3d from a YAML node, with fallback.
    static Vector3d readVec3(const YAML::Node& node, const char* key, const Vector3d& fallback);

public:
    // Chain's identifier and type
    std::string name_;
    std::string type_;
    bool is_enabled_;

    // Chain-specific parameters
    Vector3d initial_attitude_;
    Vector3d antlever_;
    Preintegration::PreintegrationOptions preintegration_options_;
    std::shared_ptr<IntegrationParameters> parameters_;
    Vector3d initial_vel_{Vector3d::Zero()};
    Vector3d initial_bg_{Vector3d::Zero()};
    Vector3d initial_ba_{Vector3d::Zero()};
    double alignment_time_{0.0};
    bool has_user_yaw_{false};

    // Data I/O
    std::unique_ptr<ImuFileLoader> loader_;
    FileSaver nav_saver_;
    FileSaver err_saver_;
    
    // State and data for the optimization window
    std::deque<std::unique_ptr<IUnifiedPreintegrator>> preintegrators_;
    std::vector<IntegrationStateData> state_data_list_;
    std::vector<IntegrationState> state_list_;

    // Temporary buffer for IMU measurements
    IMU imu_pre_, imu_cur_;
};

#endif // OB_GINS_IMU_CHAIN_H
