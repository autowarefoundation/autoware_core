# autoware_command_gate

A minimal gateway that exposes operation-mode change services and publishes matching state and gear commands.

## Features
- Services `/api/operation_mode/change_to_stop` and `/api/operation_mode/change_to_autonomous` (`autoware_adapi_v1_msgs/srv/ChangeOperationMode`).
- Publishes `/api/operation_mode/state` (reliable, transient local QoS) and `/control/command/gear_cmd` on each service call.
- STOP -> gear `PARK`; AUTONOMOUS -> gear `DRIVE`.

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
```
Echo topics:
```bash
ros2 topic echo /api/operation_mode/state
ros2 topic echo /control/command/gear_cmd
```
