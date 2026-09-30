# autoware_command_gate

A system-layer command gate that manages vehicle operation mode transitions and control authority gating.

## Overview

`autoware_command_gate` implements a **two-axis command gate** that cleanly separates the system operation mode from the Autoware vehicle control authorization flag:

1. **Operation Mode Axis**: Governs the operational state of the vehicle (`STOP`, `AUTONOMOUS`, `LOCAL`, `REMOTE`).
2. **Autoware Control Axis**: Toggles whether Autoware is actively authorized to command vehicle actuation (`autoware_control: true / false`).

This separation decouples system-level arbitration from high-level AD API state handling (which is managed by `autoware_default_adapi::OperationModeNode`).

## Features

- **Services**:
  - `/system/operation_mode/change_operation_mode` (`autoware_system_msgs/srv/ChangeOperationMode`): Requests mode transition to `STOP`, `AUTONOMOUS`, `LOCAL`, or `REMOTE`.
  - `/system/operation_mode/change_autoware_control` (`autoware_system_msgs/srv/ChangeAutowareControl`): Enables or disables Autoware control authority.
- **Publications**:
  - `/system/operation_mode/state` (`autoware_adapi_v1_msgs/msg/OperationModeState`): Publishes the authoritative system operation mode state (mode, control flag, availability).
  - `/control/command/gear_cmd` (`autoware_vehicle_msgs/msg/GearCommand`): Automatically dispatches gear commands based on the current mode:
    - `STOP` -> `PARK`
    - `AUTONOMOUS` -> `DRIVE`
    - `LOCAL` / `REMOTE` -> `NONE`

## Build

```bash
colcon build --packages-select autoware_command_gate --symlink-install
```

## Run

Launch as a component node:

```bash
ros2 launch autoware_command_gate autoware_command_gate.launch.py
```

Or run directly:

```bash
ros2 run autoware_command_gate autoware_command_gate_exe
```

## Interact

Call system services (example from a sourced workspace):

```bash
# Change mode to AUTONOMOUS (mode enum: 1=STOP, 2=AUTONOMOUS, 3=LOCAL, 4=REMOTE)
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}"

# Enable Autoware control
ros2 service call /system/operation_mode/change_autoware_control autoware_system_msgs/srv/ChangeAutowareControl "{autoware_control: true}"

# Return to STOP mode
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 1}"
```

Echo topics:

```bash
ros2 topic echo /system/operation_mode/state
ros2 topic echo /control/command/gear_cmd
```

## Tests

Run integration and unit tests:

```bash
colcon test --packages-select autoware_command_gate --event-handlers console_direct+
colcon test-result --verbose
```
