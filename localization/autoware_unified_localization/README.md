# autoware_unified_localization

Unified localization: **ROS-free Core** (EKF fusion + stop_filter + twist2accel) and **Adapter node** that subscribes to pose/twist, runs the Core, and publishes odometry and acceleration.

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

## Adapter node (ROS 2)

The **localization_node** subscribes to pose and twist (same topic names as `ekf_localizer` for drop-in use), calls the Core `FusionPipeline`, and publishes:

- **kinematic_state** (`nav_msgs/msg/Odometry`) — fused pose + twist (component_interface_specs: `/localization/kinematic_state` when remapped)
- **acceleration** (`geometry_msgs/msg/AccelWithCovarianceStamped`) — from twist2accel
- TF: `pose_frame_id` → `child_frame_id` (default: map → base_link)
- **Service** `/localization/initialize` (`autoware_localization_msgs/srv/InitializeLocalization`) — set initial pose via API (DIRECT method only: pass `pose_with_covariance` in the request).

### Topics (default names, remappable via launch)

| Role   | Topic name                   | Message type                              |
|--------|------------------------------|-------------------------------------------|
| Input  | `initialpose`                | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| Input  | `in_pose_with_covariance`   | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| Input  | `in_twist_with_covariance`   | `geometry_msgs/msg/TwistWithCovarianceStamped` |
| Output | `kinematic_state`            | `nav_msgs/msg/Odometry`                   |
| Output | `acceleration`               | `geometry_msgs/msg/AccelWithCovarianceStamped` |

### Service: set initial pose (DIRECT)

Compatible with component_interface_specs `/localization/initialize`. Only the **DIRECT** method is supported: the request must contain at least one `pose_with_covariance` (e.g. `PoseWithCovarianceStamped`); the first one is used to initialize the pipeline.

Example (after the node is running):

```bash
ros2 service call /localization/initialize autoware_localization_msgs/srv/InitializeLocalization \
  "{method: 2, pose_with_covariance: [{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.01]}}]}"
```

(`method: 2` is DIRECT; AUTO and others are not supported and return an error.)

### Run

```bash
source install/setup.bash
ros2 run autoware_unified_localization localization_node --ros-args --params-file install/autoware_unified_localization/share/autoware_unified_localization/config/unified_localization.param.yaml
```
(Use the path to the installed config after building; or use the launch file below to load it automatically.)

Or use the launch file (loads the package config by default):

```bash
source install/setup.bash
ros2 launch autoware_unified_localization unified_localization.launch.py
```

To match existing localization launch remaps (e.g. from NDT and gyro_odometer), pass input/output topic names:

```bash
ros2 launch autoware_unified_localization unified_localization.launch.py \
  input_pose_with_covariance:=/ndt_pose \
  input_twist_with_covariance:=/vehicle/twist \
  output_kinematic_state:=/localization/kinematic_state \
  output_acceleration:=/localization/acceleration
```

### Testing the Adapter node

#### Option A: Manual test (two terminals)

1. **Terminal 1** — start the node (with params):

   ```bash
   source install/setup.bash
   ros2 launch autoware_unified_localization unified_localization.launch.py
   ```

2. **Terminal 2** — send initial pose, then pose and twist a few times so the pipeline publishes:

   ```bash
   source install/setup.bash
   # Initial pose (required once)
   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.01]}}"

   # Pose (repeat a few times or use --rate 10)
   ros2 topic pub --rate 10 /in_pose_with_covariance geometry_msgs/msg/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.01]}}"

   # Twist (in another terminal or in the same with --rate)
   ros2 topic pub --rate 10 /in_twist_with_covariance geometry_msgs/msg/TwistWithCovarianceStamped \
     "{header: {frame_id: 'base_link'}, twist: {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.01]}}"
   ```

3. **Check output** (in another terminal):

   ```bash
   ros2 topic echo /kinematic_state    # Odometry
   ros2 topic echo /acceleration       # AccelWithCovarianceStamped
   ros2 run tf2_ros tf2_echo map base_link
   ```

If you use the launch file without remaps, the node’s topic namespace is its node name: use `/localization_node/kinematic_state` and `/localization_node/acceleration` (and remap `initialpose` / `in_pose_with_covariance` / `in_twist_with_covariance` under that namespace if needed). Use `ros2 topic list` to see the actual names.

#### Option B: Automated launch test

Build with tests and run the launch test (starts the node, sends initial pose + pose/twist, checks that `kinematic_state` is published):

```bash
source install/setup.bash
colcon build --packages-select autoware_unified_localization --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select autoware_unified_localization --event-handlers console_direct+
```

To run only the launch test:

```bash
colcon test --packages-select autoware_unified_localization --ctest-args -R test_test_unified_localization_launch
```

#### Option C: Test with existing localization (rosbag / live)

If you have a rosbag or a running stack that publishes NDT pose and vehicle twist:

- Remap inputs to those topics and run the unified node instead of `ekf_localizer`; then check `kinematic_state` and `acceleration` (and TF) as above.

## Design

(WIP)
