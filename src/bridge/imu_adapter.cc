/*
 * IMU adapter 实现：当前以主IMU链路为准；后续在此集中接入轮IMU。
 */

#include "imu_adapter.h"

namespace Adapter {

static WheelIntegrationParameters ToWheelParams(const std::shared_ptr<IntegrationParameters> &p) {
    WheelIntegrationParameters w{};
    w.acc_vrw      = p->acc_vrw;
    w.gyr_arw      = p->gyr_arw;
    w.gyr_bias_std = p->gyr_bias_std;
    w.acc_bias_std = p->acc_bias_std;
    w.gyr_scale_std = p->gyr_scale_std;
    w.acc_scale_std = p->acc_scale_std;
    w.corr_time    = p->corr_time;
    w.gravity      = p->gravity;
    w.odo_std      = p->odo_std;
    w.odo_srw      = p->odo_srw;
    w.abv          = p->abv;
    w.lodo         = p->lodo;
    w.station      = p->station;
    return w;
}

static WheelIntegrationState ToWheelState(const IntegrationState &s) {
    WheelIntegrationState w{};
    w.time = s.time;
    w.p    = s.p;
    w.q    = s.q;
    w.v    = s.v;
    w.bg   = s.bg;
    w.ba   = s.ba;
    w.s    = s.s;
    w.sodo = s.sodo;
    w.abv  = s.abv;
    w.sg   = s.sg;
    w.sa   = s.sa;
    return w;
}

std::shared_ptr<UnifiedPreintegrator>
UnifiedPreintegrator::Create(const std::shared_ptr<IntegrationParameters> &params, const IMU &first_imu,
                             const IntegrationState &init_state, Options opt, bool is_wheel) {
    auto up = std::shared_ptr<UnifiedPreintegrator>(new UnifiedPreintegrator(is_wheel));

    // 为保持主流程稳定，默认仍走主IMU预积分；
    // 后续如需切换到轮IMU，可在此根据 is_wheel 创建 WheelPreintegration。
    up->preint_main_ = Preintegration::createPreintegration(params, first_imu, init_state, opt);

    // 预留：构建轮IMU预积分（暂不接入主流程参数块）
    // if (is_wheel) {
    //     auto wparams = std::make_shared<WheelIntegrationParameters>(ToWheelParams(params));
    //     auto wstate  = ToWheelState(init_state);
    //     up->preint_wheel_ = WheelPreintegration::createPreintegration(wparams, first_imu, wstate,
    //                                                                   WheelPreintegration::getOptions(
    //                                                                       (opt & Preintegration::PREINTEGRATION_ODO) != 0,
    //                                                                       (opt & Preintegration::PREINTEGRATION_EARTH) != 0));
    // }

    return up;
}

void UnifiedPreintegrator::addNewImu(const IMU &imu) {
    if (preint_main_) preint_main_->addNewImu(imu);
    // if (preint_wheel_) preint_wheel_->addNewImu(imu);
}

double UnifiedPreintegrator::endTime() const {
    if (preint_main_) return preint_main_->endTime();
    // if (preint_wheel_) return preint_wheel_->endTime();
    return 0.0;
}

IntegrationState UnifiedPreintegrator::currentStateMain() const {
    if (preint_main_) return preint_main_->currentState();
    return {};
}

} // namespace Adapter

