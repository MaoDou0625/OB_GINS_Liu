# B-Spline Trajectory Module

This module implements a Continuous-Time Trajectory representation using Cumulative B-Splines on SE(3).

## Key Components

- **ControlPoint**: Holds timestamp and `Sophus::SE3d` pose (plus bias terms).
- **BSplineEvaluator**: Performs analytical evaluation of Pose, Velocity, and Acceleration at any time `t` given 4 control points.
- **SplinePoseFactor**: Ceres CostFunction for constraining the spline to measured poses.
- **SophusSE3Manifold**: Custom Ceres Manifold to handle `Sophus::SE3` parameterization correctly (Quaternion + Translation) with `AutoDiffManifold`.

## Usage

1. Initialize `ControlPoint`s at fixed time intervals `dt` (e.g., 0.1s).
2. For measurements at time `t`, identify the 4 relevant control points covering the interval.
3. Add `SplinePoseFactor` or other inertial factors to the Ceres problem.
4. Use `ceres::AutoDiffManifold<SophusSE3Plus, 7, 6>` for the SE3 parameter blocks.

## Example

See `examples/test_spline_fitting.cpp` for a complete example of fitting a spline to ground truth poses.
