# OB_GINS_CT: 多 IMU 紧耦合连续时间解算框架

本项目是一个基于 B-Spline 连续时间轨迹的 GNSS/多 IMU（标准 IMU + 轮式 IMU）紧耦合解算框架。针对轮式 IMU 的特性进行了专门建模，支持在线估计安装误差和杆臂。

## ⚠️ 当前状态：ContinuousTime 分支 (WIP)

**注意：当前仓库中的代码处于架构重构后的初始阶段，尚未经过编译验证和实测。**

## 核心特性
- **多 IMU 支持**：支持同时接入标准 IMU 和多个轮式 IMU。
- **轮式 IMU 建模**：
  - 自动处理左右轮安装侧导致的符号差异。
  - 动态计算轮心杆臂补偿。
  - 在线估计 IMU 相对于车体的安装旋转误差 ($q_{body\_imu}$)。
- **误差模型**：
  - 零偏（Bias）采用一阶高斯-马尔可夫（GM）过程建模，支持相关时间（Correlation Time）配置。
  - 支持安装参数和轮径的先验约束。
- **连续时间框架**：基于 Ceres Solver 和 B-Spline 轨迹，实现高频传感器在任意时刻的约束。

## 编译要求
- C++ 20
- CMake 3.10+
- Eigen 3
- Ceres Solver
- Sophus
- yaml-cpp
- glog

## 待办事项
- [ ] 完成首轮编译调试。
- [ ] 验证 `WheelSpeedFactor` 的雅可比矩阵正确性。
- [ ] 使用 WID 数据集进行闭环测试。