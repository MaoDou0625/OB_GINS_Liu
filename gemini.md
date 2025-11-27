# Gemini 发现

本文档记录了Gemini在分析和修改此代码库时的重要发现。

## 1. 初始姿态设置

IMU的初始姿态在 `src/core/imu_chain.cc` 中设置，其逻辑遵循以下优先级：

1.  **IMU特定 `initatt`**: 如果在 `config/ob_gins.yaml` 中为某个IMU（如 `imu_main`）单独定义了 `initatt: [R, P, Y]`，则直接使用这些横滚、俯仰、航向值。
2.  **IMU特定 `init_yaw`**: 如果没有定义 `initatt`，但定义了 `init_yaw: Y`，则该IMU的初始横滚和俯仰将为0，航向使用指定值。
3.  **全局 `initatt`**: 如果以上两者都未在IMU特定配置中定义，则使用YAML文件根目录下的全局 `initatt` 值。

## 2. 地球模型

- **地球自转角速度**: 在 `src/common/earth.h` 中被定义为常量 `WGS84_WIE`。
  ```cpp
  const double WGS84_WIE = 7.2921151467E-5; // rad/s
  ```
- 提供了辅助函数 `iewe()` 和 `iewn(lat)` 将其转换为地球坐标系（e-frame）和导航坐标系（n-frame）下的矢量表示。

## 3. 欧拉角旋转顺序

- 欧拉角的处理函数位于 `src/common/rotation.h`。
- 系统统一使用 **ZYX** 旋转顺序。
- `euler2matrix` 函数的实现明确了这一点，旋转矩阵由 `Rz * Ry * Rx` 构成，其中向量 `euler` 的元素对应 `[Roll(Rx), Pitch(Ry), Yaw(Rz)]`。
