#ifndef OB_GINS_CORE_IMU_PROCESSOR_H
#define OB_GINS_CORE_IMU_PROCESSOR_H

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <ceres/ceres.h>

#include "src/common/types.h"
#include "src/spline/BSplineEvaluator.h"

namespace ob_gins {

// 辅助函数：为时间 t 寻找控制点索引
int findControlPointIndex(double t, double t0, double dt, int max_idx);

// 抽象IMU处理器基类
class ImuProcessor {
public:
    virtual ~ImuProcessor() = default;

    virtual bool LoadConfig(const YAML::Node& config_node, const std::string& imu_name) = 0;
    virtual bool LoadData(double t_start, double t_end) = 0;
    virtual void AddFactors(ceres::Problem& problem, 
                            std::vector<spline::ControlPoint>& control_points, 
                            double spline_dt, double t0_spline,
                            const Eigen::Vector3d& gravity_vec, 
                            const Eigen::Vector3d& omega_ie_local) = 0;
    
    virtual void AddBiasFactors(ceres::Problem& problem, 
                                std::vector<spline::ControlPoint>& control_points, 
                                double spline_dt) = 0;

    const std::vector<IMU>& GetImuData() const { return valid_imu_data_; }
    const std::string& GetName() const { return name_; }

    double* GetLeverArmData() { return l_body_sensor_.data(); }

    void SaveErrors(const std::string& output_path, const std::vector<spline::ControlPoint>& control_points, double spline_dt, double t_start_global);

protected:
    std::string name_;
    std::string file_path_;
    int columns_;
    double rate_hz_;
    std::vector<IMU> all_imu_data_;
    std::vector<IMU> valid_imu_data_;
    Eigen::Vector3d l_body_sensor_ = Eigen::Vector3d::Zero();

    double acc_noise_ = 1.0e-2;
    double gyr_noise_ = 1.0e-3;
    double acc_bias_rw_ = 1.0e-4;
    double gyr_bias_rw_ = 1.0e-5;
    double acc_corr_time_ = 3600.0; // 默认 1 小时
    double gyr_corr_time_ = 3600.0;

    bool LoadImuFileAndFilter(double t_start, double t_end);
    Eigen::Vector3d LoadLeverArm(const YAML::Node& config_node, const std::string& key);
    void LoadExtrinsics(const YAML::Node& config_node);
    void LoadImuNoise(const YAML::Node& config_node);

    // IMU biases (one per control point)
    std::vector<Eigen::Vector3d> bg_;
    std::vector<Eigen::Vector3d> ba_;

    // Extrinsics: Rotation from Body to IMU
    Eigen::Quaterniond q_body_imu_initial_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_body_imu_ = Eigen::Quaterniond::Identity();
};

// 标准IMU处理器
class StandardImuProcessor : public ImuProcessor {
public:
    bool LoadConfig(const YAML::Node& config_node, const std::string& imu_name) override;
    bool LoadData(double t_start, double t_end) override;
    void AddFactors(ceres::Problem& problem, 
                    std::vector<spline::ControlPoint>& control_points, 
                    double spline_dt, double t0_spline,
                    const Eigen::Vector3d& gravity_vec, 
                    const Eigen::Vector3d& omega_ie_local) override;

    void AddBiasFactors(ceres::Problem& problem, 
                        std::vector<spline::ControlPoint>& control_points, 
                        double spline_dt) override;
};

// 轮式IMU处理器
class WheelImuProcessor : public ImuProcessor {
public:
    bool LoadConfig(const YAML::Node& config_node, const std::string& imu_name) override;
    bool LoadData(double t_start, double t_end) override;
    void AddFactors(ceres::Problem& problem, 
                    std::vector<spline::ControlPoint>& control_points, 
                    double spline_dt, double t0_spline,
                    const Eigen::Vector3d& gravity_vec, 
                    const Eigen::Vector3d& omega_ie_local) override;

    void AddBiasFactors(ceres::Problem& problem, 
                        std::vector<spline::ControlPoint>& control_points, 
                        double spline_dt) override;

private:
    std::string side_;
    Eigen::Vector3d l_sensor_odopoint_;
    
    double wheel_radius_ = 0.3;
    double speed_weight_ = 1.0;
    double nhc_weight_ = 1.0;
};

} // namespace ob_gins
#endif // OB_GINS_CORE_IMU_PROCESSOR_H