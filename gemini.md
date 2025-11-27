# Gemini's Findings for OB_ReConstruct

This document records key findings and analyses performed by Gemini for easier reference in future interactions.

## IMU Initial Attitude Configuration

The initial attitude for an IMU chain is determined from the `.yaml` configuration file. The setting is read in the `ImuChain` class constructor (`src/core/imu_chain.cc`).

The value is determined based on the following priority order:

1.  **Chain-Specific `initatt` (Highest Priority)**:
    *   **Key**: `initatt`
    *   **Location**: Under a specific IMU block (e.g., `imu_main:`).
    *   **Format**: A 3-element array `[roll, pitch, yaw]`.
    *   **Unit**: Degrees.

2.  **Chain-Specific `init_yaw`**:
    *   **Key**: `init_yaw`
    *   **Location**: Under a specific IMU block.
    *   **Format**: A single number representing the yaw. Roll and pitch are assumed to be 0.
    *   **Unit**: Degrees.

3.  **Global `initatt` (Lowest Priority)**:
    *   **Key**: `initatt`
    *   **Location**: At the root level of the YAML file.
    *   **Format**: A 3-element array `[roll, pitch, yaw]`.
    *   **Unit**: Degrees.

4.  **Default**:
    *   If none of the above keys are found, the initial attitude defaults to `[0, 0, 0]`.

### Example from `config/ob_gins.yaml`:

- A global `initatt: [ 0, 0, 0 ]` exists at the root level.
- The `imu_main` block contains its own `init_yaw: -93.43`.
- **Result**: The `imu_main` chain will use an initial attitude of `[0, 0, -93.43]` degrees, because the chain-specific `init_yaw` key overrides the global `initatt` setting.

---

## Earth's Rotation Calculation

The calculation of the Earth's rotation angular velocity is defined in `src/common/earth.h`.

- **Constant**: The scalar magnitude is defined as `const double WGS84_WIE = 7.2921151467E-5;` (in rad/s).
- **Function**: The vector in the local navigation frame (NED: North-East-Down) is calculated by the static function `Earth::iewn(double lat)`, which returns:
  - North: `WGS84_WIE * cos(lat)`
  - East: `0`
  - Down: `-WGS84_WIE * sin(lat)`

## Euler Angle Convention

The convention for Euler angles is defined in `src/common/rotation.h`.

- **Rotation Order**: The order is **ZYX (intrinsic)**. Rotations are applied in this sequence:
  1.  **Roll** (横滚): around the X-axis, using `euler[0]`.
  2.  **Pitch** (俯仰): around the Y-axis, using `euler[1]`.
  3.  **Yaw** (偏航): around the Z-axis, using `euler[2]`.
- **Confirmation**: This is explicitly commented and implemented in the `Rotation::euler2matrix` function. The resulting matrix `C_b^n` transforms vectors from the body frame to the navigation frame.