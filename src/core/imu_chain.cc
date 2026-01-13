#include "imu_chain.h"
#include "src/common/logging.h"
#include "src/common/angle.h"
#include "src/common/debug.h"
#include "src/core/preintegration_factory.h"
#include "src/factors/marginalization_info.h"
#include "src/factors/pose_manifold.h"
#include "src/factors/residual_block_info.h"

#include <algorithm>
#include <cctype>
#include <cmath>

// Helper to safely parse boolean values from YAML, supporting various formats.
static bool yamlToBool(const YAML::Node &node, bool def = false) {
    try {
        if (!node || node.IsNull()) return def;
        if (node.IsScalar()) {
            try {
                std::string t = node.as<std::string>();
                std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return (char) std::tolower(c); });
                if (t == "true" || t == "1" || t == "yes" || t == "on") return true;
                if (t == "false" || t == "0" || t == "no" || t == "off") return false;
            } catch (...) { /* fallback to other types */ }
            try { int vi = node.as<int>(); return vi != 0; } catch (...) {}
            try { return node.as<bool>(); } catch (...) { return def; }
        } else {
            try { return node.as<bool>(); } catch (...) { return def; }
        }
    } catch (...) { return def; }
    return def;
}


Vector3d ImuChain::readVec3(const YAML::Node& node, const char* key, const Vector3d& fallback) {
    try {
        if (node && node[key]) {
            auto v = node[key].as<std::vector<double>>();
            if (v.size() >= 3) return Vector3d(v.data());
        }
    } catch (...) {}
    return fallback;
}

ImuChain::ImuChain(std::string name, const YAML::Node& chain_node, const YAML::Node& global_config)
    : name_(std::move(name)), 
      type_("standard"), // Default to standard
      is_enabled_(false), 
      parameters_(std::make_shared<IntegrationParameters>()) {

    is_enabled_ = true;
    try {
        if (chain_node["type"]) {
            type_ = chain_node["type"].as<std::string>();
        }
    } catch (const YAML::Exception& e) {
        LOG(WARNING) << "[Chain] Failed to parse 'type' for chain '" << name_
                     << "'. Defaulting to 'standard'. Error: " << e.what();
        type_ = "standard";
    }
    
    // --- Load Chain-Specific or Global Parameters ---
    
    // Initial attitude
    Vector3d global_initatt = readVec3(global_config, "initatt", Vector3d::Zero()) * D2R;
    initial_attitude_ = global_initatt;
    
    // Check if global config provided specific yaw/attitude
    if (global_config["initatt"] || global_config["init_yaw"]) has_user_yaw_ = true;

    if (chain_node["initatt"]) {
        initial_attitude_ = readVec3(chain_node, "initatt", global_initatt);
        initial_attitude_ *= D2R;
        has_user_yaw_ = true;
    } else if (chain_node["init_yaw"]) {
        initial_attitude_ = Vector3d(0, 0, chain_node["init_yaw"].as<double>() * D2R);
        has_user_yaw_ = true;
    }
    
    // Lever arms and body angle
    Vector3d global_antlever = readVec3(global_config, "antlever", Vector3d::Zero());
    Vector3d global_odolever = readVec3(global_config, "odolever", Vector3d::Zero());
    Vector3d global_bodyangle = readVec3(global_config, "bodyangle", Vector3d::Zero());
    
    antlever_ = readVec3(chain_node, "antlever", global_antlever);
    if (!chain_node["antlever"]) antlever_ = readVec3(chain_node, "GNSSLA", antlever_);

    parameters_->lodo = readVec3(chain_node, "odolever", global_odolever);
    if (!chain_node["odolever"]) parameters_->lodo = readVec3(chain_node, "WheelLA", parameters_->lodo);

    parameters_->abv = readVec3(chain_node, "bodyangle", global_bodyangle);
    if (!chain_node["bodyangle"]) parameters_->abv = readVec3(chain_node, "MisalignAngle", parameters_->abv);
    parameters_->abv *= D2R;

    // Initial velocity/biases (global)
    try { auto v = global_config["initvel"].as<std::vector<double>>(); if (v.size() >= 3) initial_vel_ = Vector3d(v.data()); } catch (...) {}
    try { auto v = global_config["initgb"].as<std::vector<double>>(); if (v.size() >= 3) initial_bg_ = Vector3d(v.data()) * D2R / 3600.0; } catch (...) {}
    try { auto v = global_config["initab"].as<std::vector<double>>(); if (v.size() >= 3) initial_ba_ = Vector3d(v.data()) * 1.0e-5; } catch (...) {}
    if (global_config["aligntime"]) {
        alignment_time_ = global_config["aligntime"].as<double>();
    }

    // --- IMU Noise Parameters ---
    YAML::Node imunoise_node = chain_node["imunoise"] ? chain_node["imunoise"] : global_config["imunoise"];
    if(imunoise_node) {
        parameters_->gyr_arw      = (imunoise_node["arw"].as<double>()) * D2R / 60.0;
        parameters_->acc_vrw      = (imunoise_node["vrw"].as<double>()) / 60.0;
        parameters_->gyr_bias_std = (imunoise_node["gbstd"].as<double>()) * D2R / 3600.0;
        parameters_->acc_bias_std = (imunoise_node["abstd"].as<double>()) * 1.0e-5;
        parameters_->corr_time    = (imunoise_node["corrtime"].as<double>()) * 3600.0;
        if (imunoise_node["gsstd"]) parameters_->gyr_scale_std = imunoise_node["gsstd"].as<double>() * 1e-6;
        if (imunoise_node["asstd"]) parameters_->acc_scale_std = imunoise_node["asstd"].as<double>() * 1e-6;
    }
    // odometer noise and options
    if (global_config["odometer"]) {
        try {
            auto odo = global_config["odometer"];
            auto stdv = odo["std"].as<std::vector<double>>();
            if (stdv.size() >= 3) parameters_->odo_std = Vector3d(stdv.data());
            parameters_->odo_srw = odo["srw"].as<double>() * 1e-6;
        } catch (...) {}
    }

    // --- File I/O Setup ---
    std::string imu_path;
    int imu_datalen = 0;
    int imu_datarate = 0;
    if (chain_node["file"])      imu_path     = chain_node["file"].as<std::string>();
    if (chain_node["columns"])   imu_datalen  = chain_node["columns"].as<int>();
    if (chain_node["rate_hz"])   imu_datarate = chain_node["rate_hz"].as<int>();

    loader_ = std::make_unique<ImuFileLoader>(imu_path, imu_datalen, imu_datarate, (type_ == "wheel"));
    if (!loader_->isOpen()) {
        LOG(ERROR) << "[IO] Failed to open IMU file for chain '" << name_ << "': " << imu_path;
        is_enabled_ = false;
    }

    std::string outputpath = global_config["outputpath"].as<std::string>();
    bool use_primary_name = (name_ == "main" || name_ == "imu_main");
    {
        std::string suffix = use_primary_name ? "" : "_" + name_;
        std::string nav_path = outputpath + "/OB_GINS_TXT" + suffix + ".nav";
        std::string err_path = outputpath + "/OB_GINS_IMU_ERR" + suffix + ".bin";
        if (!nav_saver_.open(nav_path, 11, FileSaver::TEXT) || !err_saver_.open(err_path, 7, FileSaver::BINARY)) {
            LOG(WARNING) << "[IO] Failed to open output files for chain '" << name_ << "'";
        } else {
            LOG(INFO) << "[IO] Output opened for chain '" << name_ << "': nav='" << nav_path << "' err='" << err_path << "'";
        }
    }

    // --- Preintegration Options ---
    bool isuseodo = yamlToBool(global_config["odometer"]["isuseodo"], false);
    bool isearth = yamlToBool(global_config["isearth"], true);
    preintegration_options_ = Preintegration::getOptions(isuseodo, isearth);

    int windows = global_config["windows"].as<int>();
    state_data_list_.resize(windows + 1);
    state_list_.resize(windows + 1);

    LOG(INFO) << "[Chain] Initialized chain '" << name_ << "' with type '" << type_ << "': enabled=" << is_enabled_;
}

void ImuChain::alignAndSync(double start_time, const Vector3d& blh) {
    if (!is_enabled_) return;

    // First, advance the stream to the specified start_time
    do {
        imu_pre_ = imu_cur_;
        imu_cur_ = loader_->next();
    } while (imu_cur_.time < start_time && !loader_->isEof());

    // If aligntime is configured, perform static alignment using data from this point
    if (alignment_time_ > 0.0) {
        DBG_LOG(1, "ALIGN_START", "chain=" << name_ << " at t=" << start_time << " for " << alignment_time_ << "s");
        Vector3d accumulated_gyro = Vector3d::Zero();
        Vector3d accumulated_accel = Vector3d::Zero();
        int imu_count = 0;
        
        double align_end_time = start_time + alignment_time_;

        while (imu_cur_.time < align_end_time && !loader_->isEof()) {
            if (imu_cur_.dt > 1e-9) { // Avoid division by zero
                accumulated_gyro += imu_cur_.dtheta / imu_cur_.dt;
                accumulated_accel += imu_cur_.dvel / imu_cur_.dt;
                imu_count++;
            }
            imu_pre_ = imu_cur_;
            imu_cur_ = loader_->next();
        }

        if (imu_count > 0) {
            Vector3d avg_gyro = accumulated_gyro / imu_count;
            Vector3d avg_accel = accumulated_accel / imu_count;

            // Dual-vector alignment
            double roll = atan2(-avg_accel.y(), -avg_accel.z());
            double pitch = atan2(avg_accel.x(), sqrt(avg_accel.y() * avg_accel.y() + avg_accel.z() * avg_accel.z()));

            Matrix3d C_b_n_rp = Rotation::euler2matrix(Vector3d(roll, pitch, 0));
            Vector3d w_l = C_b_n_rp * avg_gyro;
            
            double yaw = atan2(-w_l.y(), w_l.x());
            double calculated_yaw = yaw;

            if (has_user_yaw_) {
                yaw = initial_attitude_.z();
                LOG(INFO) << "[Chain] " << name_ << " alignment finished. Using configured Yaw: " << yaw * R2D << " deg (Calculated: " << calculated_yaw * R2D << " deg)";
            } else {
                 LOG(INFO) << "[Chain] " << name_ << " alignment finished. Using calculated Yaw: " << yaw * R2D << " deg";
            }

            initial_attitude_ << roll, pitch, yaw;
            
            // Debug Output Level 1
            if (Debug::on(1)) {
                Vector3d att_deg = initial_attitude_ * R2D;
                // Approximate ideal outputs for verification
                double g = Earth::gravity(blh); 
                Matrix3d C_n_b = Rotation::euler2matrix(initial_attitude_).transpose();
                // Ideal Accel (Gravity in Nav is [0,0,g], so specific force in Body is C_n^b * [0,0,-g])
                Vector3d f_b_ideal = C_n_b * Vector3d(0, 0, -g);
                
                // Ideal Gyro (Earth rotation)
                Vector3d w_ie_n = Earth::iewn(blh[0]);
                Vector3d w_b_ideal = C_n_b * w_ie_n;
                
                std::stringstream ss;
                ss << "\n[ALIGN_INFO] Chain: " << name_ << "\n";
                ss << "  Attitude (Deg): R=" << att_deg[0] << " P=" << att_deg[1] << " Y=" << att_deg[2] << "\n";
                ss << "  Rotation Matrix (Body->Nav):\n" << C_n_b.transpose() << "\n";
                ss << "  Avg Accel (m/s^2): " << avg_accel.transpose() << "\n";
                ss << "  Avg Gyro (rad/s):  " << avg_gyro.transpose() << "\n";
                ss << "  Ideal Accel:       " << f_b_ideal.transpose() << "\n";
                ss << "  Ideal Gyro:        " << w_b_ideal.transpose() << "\n";
                Debug::print(1, "ALIGN", ss.str());
            }

        } else {
            LOG(WARNING) << "[Chain] No IMU data available for alignment on chain '" << name_ << "'. Using configured attitude.";
        }
    }
}

bool ImuChain::initializeFirstState(const GNSS& initial_gnss, const Vector3d& station_origin) {
    if (!is_enabled_) return false;

    DBG_LOG(1, "INIT_ATT", "chain=" << name_ << " using RPY(deg)=" << (initial_attitude_ * R2D).transpose());

    parameters_->station = station_origin;
    parameters_->gravity = Earth::gravity(station_origin);

    Vector3d init_pos = Earth::global2local(station_origin, initial_gnss.blh);

    // Initial state
    state_list_[0] = IntegrationState{
        .time = round(initial_gnss.time),
        .p    = init_pos - Rotation::euler2quaternion(initial_attitude_) * antlever_,
        .q    = Rotation::euler2quaternion(initial_attitude_),
        .v    = initial_vel_,
        .bg   = initial_bg_,
        .ba   = initial_ba_,
        .sodo = 0.0,
        .abv  = {parameters_->abv[1], parameters_->abv[2]},
    };
    
    state_data_list_[0] = PreintegrationFactory::getInstance().stateToData(type_, state_list_[0], preintegration_options_);

    startNewPreintegration();
    return true;
}

void ImuChain::processImuUpTo(double time_boundary) {
    if (!is_enabled_) return;

    while (imu_cur_.time <= time_boundary && !loader_->isEof()) {
        preintegrators_.back()->addNewImu(imu_cur_);
        imu_pre_ = imu_cur_;
        imu_cur_ = loader_->next();
    }
    
    int isneed = isNeedInterpolation(imu_pre_, imu_cur_, time_boundary);
    if (isneed == 1) { // close to the second epoch, just add it
        preintegrators_.back()->addNewImu(imu_cur_);
        imu_pre_ = imu_cur_;
        imu_cur_ = loader_->next();
    } else if (isneed == 2) { // need interpolation
        IMU interp_imu = imu_pre_;
        imuInterpolation(imu_cur_, interp_imu, imu_cur_, time_boundary);
        preintegrators_.back()->addNewImu(interp_imu);
        imu_pre_ = interp_imu;
    }
    
    // After processing, update the state list by calling the polymorphic method
    size_t current_idx = preintegrators_.size();
    preintegrators_.back()->propagateState(state_list_[current_idx]);
    
    // Convert the C++ state to a raw data array for the optimizer
    state_data_list_[current_idx] = PreintegrationFactory::getInstance().stateToData(type_, state_list_[current_idx], preintegration_options_);
}

void ImuChain::startNewPreintegration() {
    if (!is_enabled_) return;
    const auto& last_state = state_list_[preintegrators_.size()];

    PreintegrationCreationParams p = {
        .params = parameters_,
        .first_imu = imu_pre_,
        .init_state = last_state,
        .options = preintegration_options_
    };
    
    auto preintegrator = PreintegrationFactory::getInstance().create(type_, p);
    if (!preintegrator) {
        LOG(FATAL) << "[Chain] Failed to create preintegrator for type '" << type_ << "'";
        return;
    }
    preintegrators_.emplace_back(std::move(preintegrator));
}

void ImuChain::syncStatesFromOptimizer() {
    if (!is_enabled_) return;
    auto& factory = PreintegrationFactory::getInstance();
    for (size_t i = 0; i < state_list_.size(); ++i) {
        state_list_[i] = factory.stateFromData(type_, state_data_list_[i], preintegration_options_);
    }
}

void ImuChain::addParameterBlocksTo(ceres::Problem& problem, size_t max_idx) {
    if (!is_enabled_ || preintegrators_.empty()) return;

    // Parameter block sizes are constant for a given chain type.
    // Get them once from the first preintegrator.
    const int pose_size = preintegrators_.front()->getPoseParamSize();
    const int mix_size = preintegrators_.front()->getMixParamSize();

    for (size_t k = 0; k <= max_idx; k++) {
        ceres::Manifold* manifold = new PoseManifold();
        problem.AddParameterBlock(state_data_list_[k].pose, pose_size, manifold);
        problem.AddParameterBlock(state_data_list_[k].mix, mix_size);
    }
}

void ImuChain::addImuFactorsTo(ceres::Problem& problem, size_t max_idx) {
    if (!is_enabled_) return;
    for (size_t k = 0; k < max_idx; k++) {
        auto factor = preintegrators_[k]->createImuFactor();
        problem.AddResidualBlock(factor, nullptr, 
                                 state_data_list_[k].pose, state_data_list_[k].mix,
                                 state_data_list_[k+1].pose, state_data_list_[k+1].mix);
    }
}

bool ImuChain::addGnssFactorTo(ceres::Problem& problem, const GNSS& gnss, ceres::LossFunction* loss,
                               const std::deque<double>& time_list, ceres::ResidualBlockId* out_id) {
    if (!is_enabled_) return false;
    // Find the time index corresponding to the GNSS measurement
    for (size_t i = 0; i < time_list.size(); ++i) {
        if (fabs(gnss.time - time_list[i]) < 0.001) {
            if (Debug::on(2)) {
                Debug::print(2, "GNSS_MATCH",
                             "chain=" + name_ +
                             " meas_t=" + std::to_string(gnss.time) +
                             " key_t=" + std::to_string(time_list[i]) +
                             " diff=" + std::to_string(fabs(gnss.time - time_list[i])));
            }
            auto factor = new GnssFactor(gnss, antlever_);
            auto id = problem.AddResidualBlock(factor, loss, state_data_list_[i].pose);
            if (out_id) *out_id = id;
            return true;
        }
    }
    if (Debug::on(2)) {
        Debug::print(2, "GNSS_SKIP",
                     "chain=" + name_ +
                     " meas_t=" + std::to_string(gnss.time) +
                     " no keyframe within 1ms");
    }
    return false;
}

void ImuChain::addBiasFactorTo(ceres::Problem& problem, size_t idx) {
    if (!is_enabled_) return;
    auto factor = preintegrators_[idx-1]->createImuErrorFactor();
    problem.AddResidualBlock(factor, nullptr, state_data_list_[idx].mix);
}

void ImuChain::writeResult(double time, const Vector3d& origin, size_t state_idx) {
     if (!is_enabled_ || !nav_saver_.isOpen()) return;

    if (state_idx == std::numeric_limits<size_t>::max()) state_idx = preintegrators_.size();
    if (state_idx >= state_list_.size()) return;

    const auto& state = state_list_[state_idx];
    vector<double> result;
    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    
    result.clear();
    result.push_back(0); result.push_back(time);
    result.push_back(pos[0]); result.push_back(pos[1]); result.push_back(pos[2]);
    result.push_back(state.v[0]); result.push_back(state.v[1]); result.push_back(state.v[2]);
    result.push_back(att[0]); result.push_back(att[1]); result.push_back(att[2]);
    nav_saver_.dump(result);

    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    result.clear();
    result.push_back(time);
    result.push_back(bg[0]); result.push_back(bg[1]); result.push_back(bg[2]);
    result.push_back(ba[0]); result.push_back(ba[1]); result.push_back(ba[2]);
    result.push_back(state.sodo);
    err_saver_.dump(result);
}

void ImuChain::slideWindow(const std::deque<double>& time_list, const std::deque<GNSS>& gnss_list) {
    if (!is_enabled_) return;
    
    preintegrators_.pop_front();
    
    int windows = state_list_.size() - 1;
    for (int k = 0; k < windows; k++) {
        state_data_list_[k] = state_data_list_[k + 1];
        state_list_[k]      = state_list_[k + 1];
    }
    state_data_list_[windows] = state_data_list_[windows];
    state_list_[windows]      = state_list_[windows];
}

void ImuChain::addFactorsToMarginalizationInfo(std::shared_ptr<MarginalizationInfo>& marginalization_info, const GNSS& gnss) {
    if (!is_enabled_) return;

    auto factor   = std::shared_ptr<ceres::CostFunction>(preintegrators_[0]->createImuFactor());
    auto residual = std::make_shared<ResidualBlockInfo>(
        factor, nullptr,
        std::vector<double *>{state_data_list_[0].pose, state_data_list_[0].mix,
                              state_data_list_[1].pose, state_data_list_[1].mix},
        std::vector<int>{0, 1});
    marginalization_info->addResidualBlockInfo(residual);
    
    if (fabs(gnss.time - state_list_[0].time) < 0.001) {
        auto gnss_factor = std::make_shared<GnssFactor>(gnss, antlever_);
        auto gnss_residual = std::make_shared<ResidualBlockInfo>(gnss_factor, nullptr, std::vector<double *>{state_data_list_[0].pose}, std::vector<int>{});
        marginalization_info->addResidualBlockInfo(gnss_residual);
    }
}
