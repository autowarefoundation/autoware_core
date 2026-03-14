# autoware_command_gate

A minimal gateway that exposes operation-mode change services and publishes matching state and gear commands.

## Features

- Services `/api/operation_mode/change_to_stop` and `/api/operation_mode/change_to_autonomous` (`autoware_adapi_v1_msgs/srv/ChangeOperationMode`).
- Service `/system/operation_mode/change_operation_mode` (`autoware_system_msgs/srv/ChangeOperationMode`).
- Publishes `/api/operation_mode/state` and `/system/operation_mode/state` (reliable, transient local QoS) and `/control/command/gear_cmd` on each service call.
- STOP -> gear `PARK`; AUTONOMOUS -> gear `DRIVE`; LOCAL/REMOTE -> gear `NONE`.

## Build

```bash
colcon build --packages-select autoware_command_gate --symlink-install
```

## Run

Launch as a node:

```bash
ros2 launch autoware_command_gate autoware_command_gate.launch.py
```

Or run directly:

```bash
ros2 run autoware_command_gate autoware_command_gate_exe
```

## Interact

Call services (example from a sourced workspace):

```bash
ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"
# System service values: STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 1}"
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}"
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 3}"
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 4}"
```

Echo topics:

```bash
ros2 topic echo /api/operation_mode/state
ros2 topic echo /system/operation_mode/state
ros2 topic echo /control/command/gear_cmd
```

## Tests

Tests are grouped under the following directories:

- Unit tests: test/unit
- Integration tests: test/integration

Run all tests:

```bash
colcon test --packages-select autoware_command_gate --event-handlers console_direct+
```

## Coverage

Coverage targets are available when configuring with `-DENABLE_COVERAGE=ON` and require `lcov`.
Reports are filtered to this package's sources under `src/` and `include/`.

Generate unit test coverage:

```bash
colcon build --packages-select autoware_command_gate --cmake-args -DENABLE_COVERAGE=ON
cmake --build build/autoware_command_gate --target coverage_unit
```

Generate integration test coverage:

```bash
colcon build --packages-select autoware_command_gate --cmake-args -DENABLE_COVERAGE=ON
cmake --build build/autoware_command_gate --target coverage_integration
```

Coverage reports are generated under:

- build/autoware_command_gate/coverage/unit
- build/autoware_command_gate/coverage/integration

### Codecov

This repository already provides Codecov configuration at the workspace root. In CI, upload the
generated `coverage_*.info` to Codecov to compare unit vs integration coverage for this package.
