/*
 * IMU adapter: 集中主IMU与轮式IMU的分流与接口封装。
 * 目标：主流程仅关心 IMU::is_wheel，分支判断集中在此层。
 *
 * 当前实现：为尽量少改动主流程，适配层对两类接口做统一封装，
 * 默认仍走主IMU预积分实现；后续可在此处切换到轮式预积分而不改主流程。
 */

#pragma once

#include <memory>
#include <deque>

#include "src/common/types.h"

// 主IMU预积分与因子
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"
#include "src/preintegration/imu_error_factor.h"

// 轮IMU接口（为未来接入做预留，不强依赖使用）
#include "src/wheel/integration_state_wheel.h"
#include "src/wheel/preintegration_wheel.h"
#include "src/wheel/preintegration_wheel_factor.h"
#include "src/wheel/imu_error_factor_wheel.h"

namespace Adapter {

// 统一的“预积分句柄” — 封装两套实现；当前默认使用主IMU通道。
class UnifiedPreintegrator {
public:
    using Options = Preintegration::PreintegrationOptions;

    static std::shared_ptr<UnifiedPreintegrator>
    Create(const std::shared_ptr<IntegrationParameters> &params, const IMU &first_imu,
           const IntegrationState &init_state, Options opt, bool is_wheel);

    void addNewImu(const IMU &imu);

    double endTime() const;

    // 统一访问：当前状态
    // - currentStateMain(): 若为轮式IMU，则做等价字段拷贝转换为主IMU结构返回
    // - currentStateWheel(): 仅在 isWheel()==true 时有效
    IntegrationState      currentStateMain() const;
    WheelIntegrationState currentStateWheel() const;

    // 原生指针（供因子构建）。
    std::shared_ptr<PreintegrationBase> rawMain() const { return preint_main_; }
    std::shared_ptr<WheelPreintegrationBase> rawWheel() const { return preint_wheel_; }

    bool isWheel() const { return is_wheel_; }

private:
    explicit UnifiedPreintegrator(bool is_wheel) : is_wheel_(is_wheel) {}

    // 未来如需切换到轮IMU，适配层可同时维护两套，实现按需分流。
    bool is_wheel_{false};
    std::shared_ptr<PreintegrationBase> preint_main_;
    std::shared_ptr<WheelPreintegrationBase> preint_wheel_;
};

// 统一入口：预积分配置与状态打包/解包
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

// 统一创建因子（根据 up->isWheel() 分流）。
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
