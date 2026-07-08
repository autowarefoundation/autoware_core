# autoware_component_interface_specs

This package defines the standardized component interface specifications for Autoware Core, ensuring consistent communication and interaction between various components in the Autoware autonomous driving stack.

## Purpose

The purpose of this package is to:

- Provide a single source of truth for component interface definitions
- Ensure consistency across different implementations
- Facilitate modular development and component interchangeability
- Document the communication protocols between Autoware Core components

## Structure

The package contains interface specifications for various components, including:

- Message definitions
- Service interfaces
- Action interfaces

## Usage

To use these interface specifications in your component:

1. Add this package as a dependency in your package.xml:

   ```xml
   <depend>autoware_component_interface_specs</depend>
   ```

2. Use the provided interfaces in your component code.

   ```cpp
   #include <autoware/component_interface_specs/localization.hpp>
   // Example: Creating a publisher using the interface specs
   using KinematicState = autoware::component_interface_specs::localization::KinematicState;
   rclcpp::Publisher<KinematicState::Message>::SharedPtr publisher_ =
   create_publisher<KinematicState::Message>(
   KinematicState::name,
   autoware::component_interface_specs::get_qos<KinematicState>());
   // Example: Creating a subscription using the interface specs
   auto subscriber_ = create_subscription<KinematicState::Message>(
   KinematicState::name,
   autoware::component_interface_specs::get_qos<KinematicState>(),
   std::bind(&YourClass::callback, this, std::placeholders::1));
   ```

## Versioning

Each domain namespace declares a semantic `Version` (`version.hpp`) and a `Specs` tuple listing every interface it owns. `0.x` denotes an unstable interface while the standard is stabilizing. Compatibility is decided on the `MAJOR` field only; `is_compatible` is the reference encoding of that rule for consumers that compile against this package. The deploy-time admission gate in `autoware_component_interface_admission` is a no-dependency leaf package, so it restates the same relation rather than calling `is_compatible` — the two must be changed together.

## Interface manifest

`interface_manifest.json` is a machine-readable list of every registered interface (domain, interface name, message/service type, kind, and version). It is generated from the `Specs` tuples by the `generate_interface_manifest` tool and committed to the repository as the source of truth.

Regenerate it by hand after changing any domain's specs or versions:

```bash
colcon build --symlink-install --packages-select autoware_component_interface_specs
./build/autoware_component_interface_specs/generate_interface_manifest \
  src/autoware_component_interface_specs/interface_manifest.json
```

The generator emits the same layout Prettier produces, so the committed file is stable across regenerations. Adjust the output path to match where the package lives in your workspace.
