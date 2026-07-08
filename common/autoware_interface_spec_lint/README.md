# autoware_interface_spec_lint

WARN-only static and manifest checks for the Autoware component interface specifications defined in `autoware_component_interface_specs`. This package gives fast, pre-build, human-readable warnings that complement the compile-time `all_specs_valid<>` static assertions: it catches "defined-but-unregistered" specs and manifest drift that the compiler alone does not.

In milestone M0 every check is advisory: the tool prints findings but always exits 0 with `--warn-only`, and the pre-commit hook never fails the commit. The warn-to-error ratchet is a later milestone (M2).

## Checks

| Check                    | Input                      | Flags (WARN)                                                                                                                                                                              |
| ------------------------ | -------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `interface_spec_concept` | domain headers             | a struct with a `name[]` that is neither a valid topic (`Message` + `depth` + `reliability` + `durability`) nor a valid service (`Service`)                                               |
| `spec_registered`        | domain headers             | a spec struct not listed in its namespace's `using Specs = std::tuple<...>`                                                                                                               |
| `version_consistency`    | headers + manifest         | a domain not declaring exactly one `version{...}`, a MAJOR that is not `0` (the standard is unstable at `0.x` in this wave), or a manifest version that disagrees with the header version |
| `manifest_fresh`         | generator + committed JSON | the rebuilt generator output differs from the committed `interface_manifest.json`                                                                                                         |

`interface_spec_concept`, `spec_registered` and `version_consistency` are pure-Python static analyses over the domain headers and are wired into pre-commit. `manifest_fresh` is a colcon test because it needs the built M0.1 generator binary.

## Suppression contract

A spec struct is exempt from `spec_registered` when the marker `// interface-spec-lint: not-versioned` appears on the struct's own declaration line or on the line directly above it. Use it for a topic that is deliberately not part of the versioned interface set. The marker string is a fixed contract that other packages depend on, so do not change its text.

```cpp
// interface-spec-lint: not-versioned
struct PointCloudMap {
  using Message = sensor_msgs::msg::PointCloud2;
  static constexpr char name[] = "/map/point_cloud_map";
  // ...
};
```

## Usage

```bash
# Lint the core specs include dir (defaults resolve relative to the repo root).
ament_interface_spec_lint --warn-only

# Lint an explicit directory and diff against a manifest.
ament_interface_spec_lint --warn-only \
  --spec-dir common/autoware_component_interface_specs/include/autoware/component_interface_specs \
  --manifest common/autoware_component_interface_specs/interface_manifest.json
```

### `manifest_fresh`

`manifest_fresh` needs the M0.1 generator binary. Point at it with `--generator <path>` or the `INTERFACE_MANIFEST_GENERATOR` environment variable. When neither is available the check skips gracefully with a warning and records nothing, so it never blocks a build in M0.

```bash
export INTERFACE_MANIFEST_GENERATOR=$PWD/build/autoware_component_interface_specs/generate_interface_manifest
ament_interface_spec_lint --warn-only \
  --manifest common/autoware_component_interface_specs/interface_manifest.json \
  --generator "$INTERFACE_MANIFEST_GENERATOR"
```

## Milestone status

M0 is advisory only. `manifest_fresh` records drift but returns success; the pre-commit hook prints warnings and exits 0. M2 flips both the static checks and `manifest_fresh` to hard failures.
