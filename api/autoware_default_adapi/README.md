# autoware_default_adapi

## Notes

Components that relay services must be executed by the Multi-Threaded Executor.

## Features

This package is a default implementation AD API.

- [interface](document/interface.md)
- [localization](document/localization.md)
- [routing](document/routing.md)
- Operation mode: four mode services, two control services, and `/api/operation_mode/state`.

The operation mode node relays `/system/operation_mode/state` and calls the system mode and control services. The Core command gate supplies these system services in a Core-only launch. The gate needs a vehicle control mode service to enable or disable Autoware control.

Operation mode requests return a timeout failure if the system backend does not respond within three seconds. The adapter replaces the timed-out client so later requests can reach a restarted backend. A timeout does not cancel a request that the backend has already received.

## Interface

- [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/)
