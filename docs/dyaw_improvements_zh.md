# Δω 因子与相关改进说明（OB_GINS）

本文档汇总本轮对 OB_GINS 的增量改动，包含差分航向因子（Δω）接入、门控与统一阈值、在线估计基线 b、统计输出与配对插值等内容，并给出运行与评测指引。

## 实现内容

- 差分航向因子（Δω）
  - 文件：`src/factors/diff_yaw_factor.h`
  - 残差：`r = (ωz_meas − bg_z) − (vL − vR)/b`，对 `bg_z`（`mix[5]`）求导。
  - 两种模式：
    - 仅传入 `(vL−vR)/b`，只绑定 `mix`；
    - 在线估计 `b`：传 `vL/vR` + 额外标量参数块 `b`，雅可比 `dr/db = (vL−vR)/b^2 / σ`。

- 在线估计基线 b（可冻结再释放）
  - 代码：`src/ob_gins.cc` 新增参数块与先验；`src/factors/scalar_prior_factor.h`（1D 高斯先验）。
  - YAML：`diff_yaw.{estimate_baseline, baseline_sigma_m, freeze_duration_s}`。

- 统一门控阈值（gates）与 Δω 门控模式（gate_mode）
  - 顶层 YAML：`gates: { yaw_rate: {thr, scale}, accel: {thr, scale} }`，统一覆盖 ODO/NHC/Δω。
  - Δω：`diff_yaw.gate_mode: hard | scale | off`（与旧 `hard_gate` 兼容）。

- 诊断统计与配对改进
  - 线性插值：L/R 轮速对中点时间插值，提升 Δω 时间对齐与稳定性。
  - 统计输出：
    - `out/<run>/dyaw_stats.csv`：`time_end,pairs,kept,scaled,gated,gate_mode,estimate_b,freeze_s`；
    - `out/<run>/dyaw_baseline.csv`：`time_end,baseline,freeze`（开启 b 估计时）。
  - YAML：`diff_yaw.log_stats: true`。

## 主要修改文件

- 因子与先验：
  - `src/factors/diff_yaw_factor.h`
  - `src/factors/scalar_prior_factor.h`
- 构图与解析：
  - `src/ob_gins.cc`（Δω 因子装配、b 参数块与先验、统一门控解析、统计输出、L/R 插值）
- 配置样例：
  - 30s：`config/run_car_30s_no_odo.yaml`
  - 120s 固定 b：`config/run_car_120s_dyaw_fixed.yaml`
  - 120s 估计 b（硬门控）：`config/run_car_120s_dyaw_estb.yaml`
  - 120s 估计 b（缩权门控）：`config/run_car_120s_dyaw_estb_scale.yaml`

## YAML 关键项（片段）

```yaml
gates:
  yaw_rate: { thr: 0.5, scale: 2.0 }
  accel:    { thr: 3.0, scale: 2.0 }

diff_yaw:
  enable: true
  # 权重
  sigma_radps: 0.05
  huber_delta: 1.0
  pair_tolerance_s: 0.01
  # 门控与缩权
  use_shared_odo_gates: true
  gate_mode: hard   # 或 scale / off
  # b 在线估计
  estimate_baseline: true
  baseline_m: 1.5
  baseline_sigma_m: 0.05
  freeze_duration_s: 30
  # 统计输出
  log_stats: true
```

## 运行与评测

- 构建（Windows + vcpkg + NMake）：`cmd /c build_ob_gins_win.bat`
- 运行示例：
  - 30s：`OB_GINS/bin/ob_gins.exe OB_GINS/config/run_car_30s_no_odo.yaml`
  - 120s 固定 b：`OB_GINS/bin/ob_gins.exe OB_GINS/config/run_car_120s_dyaw_fixed.yaml`
  - 120s 估计 b（硬门控）：`.../run_car_120s_dyaw_estb.yaml`
  - 120s 估计 b（缩权门控）：`.../run_car_120s_dyaw_estb_scale.yaml`
- 200 Hz 真值评测：
  - `python OB_GINS/tools/compare_nav_truth_200hz.py <out>/OB_GINS_TXT.nav 'OB_GINS/dataset/car/Ground Truth/truth_200hz.nav' --out <out>/nav_vs_truth_200hz.csv`

## 小结与当前观测

- 30s 窗：Δω 硬门控 on/off RMSE_h 差异极小（~0.001 m），稳定性略有提升；
- 120s 窗：
  - 固定 b 与 估计 b（硬门控）RMSE_h 相当（~0.854 m），但硬门控下 Δω 基本被跳过；
  - 改为缩权门控后 Δω 得以参与，但当前默认阈值与权重下 b 未出现显著更新。

## 调参与后续建议

- 若期望 b 收敛：可将 Δω 门控改为 `scale`，适度放宽阈值（如 yaw_rate thr 0.8–1.0，accel thr 4–5），并适度降低 `sigma_radps`（如 0.03）。
- 窗口延长至 120s 或更多，以观察基线收敛曲线；
- 若具备左右真实独立轮速（`odo_right.txt`），优先使用以增强 (vL−vR) 信号；
- 后续可选：引入 sL/sR 在线标定（弱先验），统一 ODO/NHC 的 `gate_mode`，并进入多 IMU 刚体/姿态/陀螺共享阶段（设计文档阶段 4–5）。

## 版本与提交

- 主要提交（示例）：
  - `feat(diff_yaw): add differential yaw-rate factor, YAML config and hard-gate option...`
  - `feat(dyaw): baseline b online estimation + stats + unified gates`
- 新文件：
  - `src/factors/diff_yaw_factor.h`
  - `src/factors/scalar_prior_factor.h`
  - `tools/compare_nav_truth_200hz.py`
  - 多个 30s/120s 配置文件

