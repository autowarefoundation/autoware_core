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

A domain declares its version, its `Specs` registry, and the ADL hook that `spec_version<Spec>()` resolves through in one macro, so the three cannot drift apart:

```cpp
namespace autoware::component_interface_specs::control
{
struct ControlCommand { /* ... */ };

AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0, ControlCommand)
}  // namespace autoware::component_interface_specs::control
```

## Quality of service

Topic specs carry the QoS their endpoints are created with (`depth`, `reliability`, `durability`); `get_qos<Spec>()` turns that into an `rclcpp::QoS`.

Services have a QoS profile too, but not a per-spec one. Every `create_service` and `create_client` call takes ROS 2's `rmw_qos_profile_services_default` — `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE` — unmodified, so all services really do run under identical conditions. Rather than repeat that profile on every `ServiceSpec`, it is declared once as `service_qos` in `utils.hpp` and returned by `get_service_qos()`. `test_service_qos.cpp` asserts it still equals the RMW default it mirrors, so a change to that default surfaces as a test failure instead of as silent drift between the specs and the wire.

## C++ standard

The domain headers, `version.hpp` and `utils.hpp` are C++17, matching the `CMAKE_CXX_STANDARD 17` that `autoware_package()` gives every Autoware target.

`concepts.hpp` needs C++20, because the standard library only exposes `<concepts>` in C++20 mode. A target that wants it opts in:

```cmake
target_compile_features(<target> PRIVATE cxx_std_20)
```

CMake raises that one target to `-std=c++20` and leaves every other target — and every C++17 consumer of the domain headers — on `-std=c++17`. This is what the package's own `generate_interface_manifest` and gtest targets do, and it builds on Humble / gcc-11 (Ubuntu 22.04) as well as on Jazzy. Both targets are `BUILD_TESTING`-gated, so a C++20 compile failure in either could never break the header-only package for its C++17 consumers.

Below C++20, `concepts.hpp` is an empty header rather than a hard error, so including it is always safe. Test `AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS` before naming anything it declares.

## Interface manifest

`interface_manifest.json` is a machine-readable list of every registered interface: its domain, interface name, message or service type, kind, version, and the QoS its endpoints are created with. It is generated from the `Specs` tuples by the `generate_interface_manifest` tool and committed to the repository as the source of truth.

```json
{
  "domain": "control",
  "interface": "/control/command/control_cmd",
  "type": "autoware_control_msgs/msg/Control",
  "kind": "topic",
  "version": "0.1.0",
  "qos": {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile"
  }
}
```

`reliability` and `durability` are the two axes ROS 2 checks when it decides whether a publisher and a subscription may talk at all, which is why a deploy-time gate needs them alongside the type and the version. Service entries carry the shared `service_qos` profile.

Regenerate the manifest by hand after changing any domain's specs, QoS or versions:

```bash
colcon build --symlink-install --packages-select autoware_component_interface_specs
./build/autoware_component_interface_specs/generate_interface_manifest \
  src/autoware_component_interface_specs/interface_manifest.json
```

The generator emits the same layout Prettier produces, so the committed file is stable across regenerations. Adjust the output path to match where the package lives in your workspace. `test_manifest.cpp` diffs the generator's output against the committed file, so a stale manifest fails the build rather than reaching consumers.
