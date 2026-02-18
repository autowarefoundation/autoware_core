# autoware_unified_localization

Unified localization: **ROS-free Core** (EKF fusion + stop_filter + twist2accel) and optional Adapter node (future).

## MVP Core (current)

- **EKF Core**: Time-delay EKF for pose + twist fusion (state: x, y, yaw, yaw_bias, vx, wz). Parameters via `EKFParams` struct.
- **Stop filter**: Zeroes linear/angular velocity when below thresholds. Parameters via `StopFilterParams`.
- **Twist2Accel**: Differentiates twist and applies 1D low-pass filter. Parameters via `Twist2AccelParams`.
- **Fusion pipeline**: `FusionPipeline` wires EKF → stop filter; twist2accel is available for acceleration output.
- All I/O uses plain structs (`PoseWithCovariance`, `TwistWithCovariance`, `OdometryOutput`, etc.); no ROS types.

### Dependencies (ROS-free for Core)

- `Eigen3`
- `autoware_kalman_filter` (Eigen only)

### Build

From the Autoware workspace (so that `autoware_kalman_filter` is available):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # if dependencies are already built
colcon build --packages-select autoware_unified_localization
```

To build with tests (requires `Threads` to be found by CMake):

```bash
colcon build --packages-select autoware_unified_localization --cmake-args -DBUILD_TESTING=ON
```

**Making Threads detectable**

- Building tests requires CMake to find the thread library (pthread).
- **Linux fallback**: If `find_package(Threads)` fails, we define `Threads::Threads` ourselves when `libpthread` is found, so tests can still be built. If packages are installed but Threads is not detected (e.g. due to cache or environment), a **clean build** may allow this fallback to take effect.
- **Clean build**: Run `rm -rf build/autoware_unified_localization install/autoware_unified_localization`, then `colcon build --packages-select autoware_unified_localization` again.
- **Debian/Ubuntu**: If still not detected, ensure `build-essential` or `libc6-dev` is installed.
- To build only the library without tests, use `-DBUILD_TESTING=OFF`.

If tests are built, **source the install space before running**:

```bash
source install/setup.bash   # load this workspace's install
ros2 run autoware_unified_localization test_fusion_pipeline
```

(Without `source`, you will get "Package 'autoware_unified_localization' not found".)

**If you get "No executable found"**: The test may not have been built. If the build log shows "Threads not found; skipping test_fusion_pipeline", the test was skipped. Do a clean build as above or see "Making Threads detectable" above.

### Usage example (C++)

```cpp
#include "autoware/unified_localization_core/fusion_pipeline.hpp"

autoware::unified_localization_core::FusionPipelineParams params;
params.ekf.predict_frequency = 50.0;
params.ekf.ekf_dt = 0.02;
autoware::unified_localization_core::FusionPipeline pipeline(params);

autoware::unified_localization_core::PoseWithCovariance initial_pose;
initial_pose.timestamp_sec = 0.0;
initial_pose.position_x = 0.0;
initial_pose.position_y = 0.0;
initial_pose.position_z = 0.0;
initial_pose.orientation_w = 1.0;
// set covariance as needed...
pipeline.initialize(initial_pose);

// Each step: predict, then optional pose/twist updates
double t = 0.02;
pipeline.step(t, 0.02, nullptr, &twist);

autoware::unified_localization_core::OdometryOutput odom;
pipeline.get_odometry(t, odom);
// use odom.position_*, odom.linear_x, odom.angular_z, etc.
```

## Design

(WIP)
