# autoware_command_gate

A gateway that stores the operation mode and Autoware control flag separately.

## Features

- Provides `/system/operation_mode/change_operation_mode` (`autoware_system_msgs/srv/ChangeOperationMode`) for STOP, AUTONOMOUS, LOCAL, and REMOTE.
- Provides `/system/operation_mode/change_autoware_control` (`autoware_system_msgs/srv/ChangeAutowareControl`) for the independent control flag. It requests the vehicle mode through `/control/control_mode_request` and returns failure if that service is unavailable, rejects the request, or takes more than two seconds.
- Publishes `/system/operation_mode/state` with reliable, transient local QoS. The initial state is STOP with control disabled, even while simulation time is paused. The control flag changes only when `/vehicle/status/control_mode` reports a new mode.
- Publishes `/control/command/gear_cmd` for each valid mode request: PARK for STOP, DRIVE for AUTONOMOUS, and NONE for LOCAL or REMOTE.

The `autoware_default_adapi` operation mode node provides the public `/api/operation_mode/*` services and state topic.
The control service needs a vehicle or simulator that provides `/control/control_mode_request` (`autoware_vehicle_msgs/srv/ControlModeCommand`).
Service success means that the vehicle accepted the request. The control flag changes when the vehicle reports its mode.
A timed-out request can still take effect at the vehicle. A later control mode report updates the state.

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

In one terminal, start the gear subscriber before a mode request. The gear topic uses volatile QoS.

```bash
ros2 topic echo /control/command/gear_cmd
```

In another terminal, call the system services from a sourced workspace. Mode values are STOP=1, AUTONOMOUS=2, LOCAL=3, and REMOTE=4.

```bash
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}"
ros2 service call /system/operation_mode/change_autoware_control autoware_system_msgs/srv/ChangeAutowareControl "{autoware_control: true}"
ros2 topic echo /system/operation_mode/state --qos-durability transient_local
```

## Tests

The ROS integration tests are in `test/integration`.

Run all tests:

```bash
colcon test --packages-select autoware_command_gate --event-handlers console_direct+
```
