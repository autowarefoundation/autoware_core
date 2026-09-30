# autoware_command_gate

A system-layer command gate that manages vehicle operation mode transitions and control authority gating.

## Overview

`autoware_command_gate` implements a **two-axis command gate** that cleanly separates the system operation mode from the Autoware vehicle control authorization flag:

1. **Operation Mode Axis**: Governs the operational state of the vehicle (`STOP`, `AUTONOMOUS`, `LOCAL`, `REMOTE`).
2. **Autoware Control Axis**: Toggles whether Autoware is actively authorized to command vehicle actuation (`autoware_control: true / false`).

This separation decouples system-level arbitration from high-level AD API state handling (which is managed by `autoware_default_adapi::CoreOperationModeNode`).

## Features

- **Services**:
  - `/system/operation_mode/change_operation_mode` (`autoware_system_msgs/srv/ChangeOperationMode`): Requests mode transition to `STOP`, `AUTONOMOUS`, `LOCAL`, or `REMOTE`.
  - `/system/operation_mode/change_autoware_control` (`autoware_system_msgs/srv/ChangeAutowareControl`): Requests the vehicle mode through `/control/control_mode_request` (`autoware_vehicle_msgs/srv/ControlModeCommand`) and returns failure if that service is unavailable, rejects the request, or takes more than two seconds.
- **Publications**:
  - `/system/operation_mode/state` (`autoware_adapi_v1_msgs/msg/OperationModeState`): Publishes the authoritative system operation mode state with reliable, transient local QoS. The initial state is STOP with control disabled, even while simulation time is paused. The control flag changes only when `/vehicle/status/control_mode` reports a new mode.
  - `/control/command/gear_cmd` (`autoware_vehicle_msgs/msg/GearCommand`): Publishes gear commands for each valid mode request:
    - `STOP` -> `PARK`
    - `AUTONOMOUS` -> `DRIVE`
    - `LOCAL` / `REMOTE` -> `NONE`

The `autoware_default_adapi` operation mode node provides the public `/api/operation_mode/*` services and state topic.
The control service needs a vehicle or simulator that provides `/control/control_mode_request`.
Service success means that the vehicle accepted the request. The control flag changes when the vehicle reports its mode.
Full autonomous control, autonomous steering only, and autonomous velocity control only all set the control flag to true.
A timed-out request can still take effect at the vehicle. A later control mode report updates the state.

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

In one terminal, start the gear subscriber before a mode request (the gear topic uses volatile QoS):

```bash
ros2 topic echo /control/command/gear_cmd
```

In another terminal, call the system services from a sourced workspace (mode values: STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4):

```bash
# Change mode to AUTONOMOUS
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}"

# Enable Autoware control
ros2 service call /system/operation_mode/change_autoware_control autoware_system_msgs/srv/ChangeAutowareControl "{autoware_control: true}"

# Echo operation mode state
ros2 topic echo /system/operation_mode/state --qos-durability transient_local
```

## Tests

The ROS integration tests are in `test/integration`.

Run all tests:

```bash
colcon test --packages-select autoware_command_gate --event-handlers console_direct+
colcon test-result --verbose
```
