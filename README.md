# autoware_core

- An [Autoware](https://github.com/autowarefoundation/autoware) repository that contains a basic set of high-quality, stable ROS packages for autonomous driving.

- Although this repository is currently empty, porting of code from Universe to Core will begin once the interfaces for Autoware Core/Universe have been finalized, as per ongoing [Autoware Architecture WG](https://github.com/autowarefoundation/autoware/discussions?discussions_q=label%3Aarchitecture_wg) discussions.
- A more detailed explanation about Autoware Core can be found on the [Autoware concepts documentation page](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/#the-core-module).

- For researchers and developers who want to extend the functionality of Autoware Core with experimental, cutting-edge ROS packages, see [Autoware Universe](https://github.com/autowarefoundation/autoware_universe).

## Simulation

We can run [the planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/) based on the `autoware_core`.

### Build Dependencies

```bash
# If you have not cloned `autoware_core` yet
$ git clone https://github.com/autowarefoundation/autoware_core.git

$ mkdir -p <your workspace>
$ cd <your workspace>/autoware_core
$ mkdir -p src \
    && vcs import src < autoware_core.repos \
    && vcs import src < simulator.repos \
    && vcs import src < tools.repos

# Currently, this command does not work
# $ rosdep install -y \
#     --from-paths \
#         ./autoware_core \
#         ./src/universe/autoware_universe/simulator/autoware_simple_planning_simulator \
#         ./src/universe/autoware_universe/simulator/autoware_dummy_perception_publisher \
#         ./src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter \
#         ./src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch \
#     --ignore-src \
#     --rosdistro $ROS_DISTRO

# Build except scenario_simulator
$ colcon build --base-paths ./ \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to \
        autoware_core \
        autoware_simple_planning_simulator \
        autoware_dummy_perception_publisher \
        autoware_raw_vehicle_cmd_converter \
        autoware_evaluation_adapter \
        sample_vehicle_description \
        sample_sensor_kit_description

# Build scenario_simulator
$ colcon build --base-paths ./ \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to scenario_simulator_v2
```

### Launch Planning Simulation

```bash
$ source install/setup.bash
$ ros2 launch autoware_core autoware_core.launch.xml \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    launch_sensing:=false \
    launch_sensing_driver:=false \
    launch_perception:=false \
    is_planning_simulation:=true \
    map_path:=PATH_TO_YOUR_MAP
```

## Scenario Tests

### Build for Planning Simulation

Finish the procedure in "Planning Simulation"

### Apply Patches

- Open `<your workspace>/autoware_core/src/simulator/scenario_simulator/test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py` and modify as follows.

```diff
diff --git a/test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py b/test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
index 2b5f645eb..20cc70aa1 100755
--- a/test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
+++ b/test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
@@ -49,7 +49,7 @@ def default_autoware_launch_package_of(architecture_type):
     return {
         "awf/universe/20230906": "autoware_launch",
         "awf/universe/20240605": "autoware_launch",
-        "awf/universe/20250130": "autoware_launch",
+        "awf/universe/20250130": "autoware_core",
     }[architecture_type]


@@ -61,7 +61,7 @@ def default_autoware_launch_file_of(architecture_type):
     return {
         "awf/universe/20230906": "planning_simulator.launch.xml",
         "awf/universe/20240605": "planning_simulator.launch.xml",
-        "awf/universe/20250130": "planning_simulator.launch.xml",
+        "awf/universe/20250130": "scenario_simulator.launch.xml",
     }[architecture_type]
```

### Launch Scenario Simulation

```bash
$ source install/setup.bash
$ ros2 launch scenario_test_runner scenario_test_runner.launch.py \
    architecture_type:=awf/universe/20250130 \
    record:=false \
    scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
    sensor_model:=sample_sensor_kit \
    vehicle_model:=sample_vehicle
```
