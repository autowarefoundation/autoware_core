# Operation Mode API

## Overview

Centralizes the vehicle operation mode state and mode change requests between external AD API clients and the internal system command gate.

The `OperationModeNode` provides standard AD API services for requesting operational mode changes and enabling/disabling Autoware control authority. It coordinates with `autoware_command_gate` via system-level interfaces (`/system/operation_mode/*`) and publishes the public, latched `/api/operation_mode/state` topic.

See the [autoware-documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/operation_mode/) for AD API specifications.

## Interfaces

| Interface    | Local Name                 | Global Name                                    | Description                                   |
| ------------ | -------------------------- | ---------------------------------------------- | --------------------------------------------- |
| Service      | ~/change_to_stop           | /api/operation_mode/change_to_stop             | Request transition to STOP mode               |
| Service      | ~/change_to_autonomous     | /api/operation_mode/change_to_autonomous       | Request transition to AUTONOMOUS mode         |
| Service      | ~/change_to_local          | /api/operation_mode/change_to_local            | Request transition to LOCAL mode              |
| Service      | ~/change_to_remote         | /api/operation_mode/change_to_remote           | Request transition to REMOTE mode             |
| Service      | ~/enable_autoware_control  | /api/operation_mode/enable_autoware_control    | Enable vehicle control by Autoware            |
| Service      | ~/disable_autoware_control | /api/operation_mode/disable_autoware_control   | Disable vehicle control by Autoware           |
| Publication  | ~/state                    | /api/operation_mode/state                      | Latched state of operation mode               |
| Subscription | -                          | /system/operation_mode/state                   | System operation mode state from command gate |
| Client       | -                          | /system/operation_mode/change_operation_mode   | System-level mode change service              |
| Client       | -                          | /system/operation_mode/change_autoware_control | System-level control change service           |
