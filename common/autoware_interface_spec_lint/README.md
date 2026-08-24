# autoware_interface_spec_lint

Config-driven static and manifest checks for the Autoware component interface specifications defined in `autoware_component_interface_specs`. This package gives fast, pre-build, human-readable diagnostics that complement the compile-time `all_specs_valid<>` static assertions: it catches "defined-but-unregistered" specs, version inconsistencies, cross-owner shadowing, heavy-raw topics that must not be versioned, and manifest drift that the compiler alone does not.

Each gate's severity comes from the committed gate config (`config/interface_gates.yaml`): `off` (not run), `warn` (advisory, prints findings but never fails), or `error` (fails the build when it reports at least one finding). Every implemented gate is set to `error`, so a finding now fails CI and the pre-commit hook; only the gates listed under "Honest deferrals" below are `off`. `--warn-only` overrides every gate to exit 0 for local advisory runs.

## Gates

| Gate                     | Input                      | Severity | Flags                                                                                                                                                                                               |
| ------------------------ | -------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `interface_spec_concept` | domain headers             | error    | a struct with a `name[]` that is neither a valid topic (`Message` + `depth` + `reliability` + `durability`) nor a valid service (`Service`)                                                         |
| `spec_registered`        | domain headers             | error    | a spec struct not listed in its namespace's `using Specs = std::tuple<...>` (unless it carries the suppression marker)                                                                              |
| `version_consistency`    | headers + manifest         | error    | a domain not declaring exactly one `version{...}`, or a manifest version that disagrees with the header version                                                                                     |
| `qos_consistency`        | headers + manifest         | error    | a registered spec whose `history` / `depth` / `reliability` / `durability` disagrees with its manifest `qos` block, is missing from the manifest, or names a QoS policy the manifest cannot express |
| `manifest_fresh`         | generator + committed JSON | error    | the rebuilt generator output differs from the committed `interface_manifest.json` (see the binding-gate note below)                                                                                 |
| `owner_isolation`        | manifest(s)                | error    | a non-base-owner manifest entry whose interface name shadows a base-owner (`autowarefoundation`) entry                                                                                              |
| `no_raw_spec_topic`      | manifest(s)                | error    | a versioned **topic** whose interface name contains a heavy-raw deny substring; services are out of scope                                                                                           |

The manifest audits (`owner_isolation`, `no_raw_spec_topic`) take a _list_ of manifest dicts so a future vendor-partition manifest can be cross-checked against the base manifest; with the single committed core manifest `owner_isolation` is vacuously clean.

### Binding manifest-freshness gate

The lint's `manifest_fresh` can only bind where the manifest generator binary is available; on a plain checkout / pre-commit run it skips gracefully. The **binding** freshness gate is the specs-package gtest (`autoware_component_interface_specs/test/test_manifest.cpp`), which regenerates the manifest with the built generator and asserts byte-equality against the committed `interface_manifest.json` on every build-and-test CI run. Manifest drift is therefore a hard failure regardless of the advisory lint.

## Honest deferrals

The following design gate-table entries are declared `off` in the committed config and are intentionally deferred; enabling one is rejected by the config loader because there is no implementation to run yet.

| Gate                       | Why deferred                                                                                                                             |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `no_foreign_if_dependency` | needs per-component provided/required role manifests, which do not exist yet (they arrive with the deploy-time OCI-label manifest track) |
| `profile_compat`           | needs a vendor-specific interface profile that does not exist in this package yet; out of scope for this change                          |
| `if_usage_coverage`        | runtime gate; needs runtime introspection data this static-analysis package does not have, deferred to future work                       |
| `admission_smoke`          | runtime gate; needs runtime introspection data this static-analysis package does not have, deferred to future work                       |

`owner_isolation` is wired and at `error` but binds only once a second (vendor-partition) manifest exists to cross-check against. Today the universe versioned surface is exactly the core re-exports (single version authority in core) and universe-owned specs are unversioned, so there is no separate universe-side gate wiring yet.

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
# Lint the core specs with the committed gate config (severities from the config).
ament_autoware_interface_spec_lint

# Advisory local run: print findings but always exit 0.
ament_autoware_interface_spec_lint --warn-only

# Explicit paths and an alternate gate config.
ament_autoware_interface_spec_lint \
  --config common/autoware_interface_spec_lint/config/interface_gates.yaml \
  --spec-dir common/autoware_component_interface_specs/include/autoware/component_interface_specs \
  --manifest common/autoware_component_interface_specs/interface_manifest.json
```

### `manifest_fresh`

`manifest_fresh` needs the manifest generator binary. Point at it with `--generator <path>` or the `INTERFACE_MANIFEST_GENERATOR` environment variable. When neither is available the check skips gracefully (the binding gate is the specs-package gtest).

```bash
export INTERFACE_MANIFEST_GENERATOR=$PWD/build/autoware_component_interface_specs/generate_interface_manifest
ament_autoware_interface_spec_lint \
  --manifest common/autoware_component_interface_specs/interface_manifest.json \
  --generator "$INTERFACE_MANIFEST_GENERATOR"
```

## Gate config

`config/interface_gates.yaml` maps each gate to a severity and carries the `no_raw_spec_topic` deny/allow lists and the `owner_isolation` base owner. The loader rejects an unknown gate name and rejects enabling a deferred gate (it may only be declared `off`).
