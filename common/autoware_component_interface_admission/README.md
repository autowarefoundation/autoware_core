# autoware_component_interface_admission

The shared **component-interface admission rule** and the **deploy-time manifest gate** for Autoware's component interface versioning. This is a standalone, ROS-message-free leaf package: it depends only on the ament build system and `nlohmann-json` (no `rclcpp`, no `autoware_component_interface_specs`), so it builds against today's released core.

## One rule, two triggers

Interface compatibility is enforced by a single admission rule — "the consumer's accepted MAJOR range contains the provider's MAJOR" plus a remap-safe two-layer name match — evaluated at two triggers:

- **Deploy-time (primary)**: each component bakes its interface manifest into its container image, and a pre-boot gate cross-checks the whole composed image set, rejecting an incompatible combination before anything is built, pulled, or booted. This package provides that gate (`evaluate_deploy()` + the `manifest_admit` CLI).
- **Runtime (secondary, not yet implemented)**: the same rule at component startup over a broadcast manifest. This package provides the rule (`evaluate()`); the runtime broadcast and checker are not implemented yet (see the deferred-work note below).

Both triggers live in `admission_rule.hpp` and share the same version-compatibility rule: the deploy trigger applies **stage 1** (version + `interface_name`), and the runtime trigger adds **stage 2** (the remap-resolved `resolved_name` match). One rule, evaluated at the depth each trigger can see — not a parallel reimplementation.

## Admission rule

For each required interface, the rule finds providers of the same `interface_name` and applies a two-layer match (`evaluate()` in `include/autoware/component_interface_admission/admission_rule.hpp`):

| Situation                                                                 | Verdict          | Code |
| ------------------------------------------------------------------------- | ---------------- | ---- |
| version-ok **and** `resolved_name` coincide (the actually-wired provider) | `ACCEPTED`       | 0    |
| MAJOR in range but `min_minor` unmet                                      | `MINOR_MISMATCH` | 2    |
| MAJOR out of the accepted range                                           | `MAJOR_MISMATCH` | 1    |
| version-ok but a remap left `resolved_name` disjoint                      | `TOPIC_MISMATCH` | 3    |
| required interface has **no provider** in the set                         | `NO_PROVIDER`    | 4    |

The MINOR bound is inclusive (`provider.minor >= min_minor`), and `min_minor == 0` means unconstrained. Because MINOR resets to 0 on every MAJOR bump (semver), `min_minor` binds **only at the MAJOR it was declared against** (`accept_major_min`); at any higher accepted MAJOR the bound is already satisfied. So a consumer accepting `[2, 3]` with `min_minor = 5` admits provider `2.5` and `3.0`, but rejects `2.4` as a `MINOR_MISMATCH`. Among several version-compatible providers, the one whose `resolved_name` coincides is preferred (the wired provider); a version-compatible provider left on a disjoint wire topic by a remap is the false-accept that logical-name-only matching would miss, reported as `TOPIC_MISMATCH`.

### Deploy vs runtime: `NO_PROVIDER` is deploy-only

The one place the two triggers differ is a required interface with no provider:

- **Runtime** (`evaluate()`): such a required interface is **skipped** — under the runtime trigger a provider may simply not have started yet, so absence is not yet a failure.
- **Deploy-time** (`evaluate_deploy()`): the image set is complete, so a required interface with no provider anywhere in the set is a hard `NO_PROVIDER` rejection.

The deploy-time gate matches on **version + `interface_name` only** (stage 1). The remap-resolved `resolved_name` match (stage 2 of the rule) is runtime-only, because remaps live in the launch / compose layer and are not visible in image metadata — so `evaluate_deploy()` never inspects `resolved_name` and never emits `TOPIC_MISMATCH`. That residual remap false-accept is exactly what the runtime trigger backstops.

## Records and JSON schema

`records.hpp` defines plain C++ structs that mirror the (future) handshake message set field-for-field, so the eventual rosidl binding is mechanical:

- `ProvidedInterface { ns, interface_name, resolved_name, type_name, major, minor, patch }`
- `RequiredInterface { ns, interface_name, resolved_name, type_name, accept_major_min, accept_major_max, min_minor }`
- `InterfaceManifest { owner, node_name, provided[], required[] }`

`interface_name` is the spec-declared name (`Spec::name`), remap-invariant and the matching key; `resolved_name` is the remap-resolved fully-qualified name, equal to `interface_name` when not remapped.

`manifest_json.hpp` serializes a manifest to / parses it from this JSON payload (the OCI-label payload schema below):

```json
{
  "owner": "autowarefoundation",
  "node_name": "/perception/detection",
  "provided": [
    {
      "ns": "perception",
      "interface_name": "/perception/object_recognition/objects",
      "resolved_name": "/perception/object_recognition/objects",
      "type_name": "autoware_perception_msgs/msg/PredictedObjects",
      "major": 2,
      "minor": 1,
      "patch": 0
    }
  ],
  "required": [
    {
      "ns": "map",
      "interface_name": "/map/vector_map",
      "resolved_name": "/map/vector_map",
      "type_name": "autoware_map_msgs/msg/LaneletMapBin",
      "accept_major_min": 1,
      "accept_major_max": 2,
      "min_minor": 0
    }
  ]
}
```

`from_json()` is **defensive**: any malformed input — a JSON syntax error, a non-object root, a missing required key, or a value of the wrong type — is reported by throwing `std::runtime_error`, never undefined behaviour or a crash. Required per entry: `interface_name` and the numeric version / range fields. Optional-with-default: `ns` / `type_name` / `owner` / `node_name` default to `""`, `resolved_name` defaults to `interface_name`, and the `provided` / `required` arrays default to empty when absent.

## Deploy-time gate: OCI label contract

Each component image carries its interface manifest as **pure image metadata**, so the gate reads it without creating or starting a container and without any source present in the image (a binary-only third-party image works):

- **Primary**: the OCI image label `org.autoware.interface_manifest`, whose value is the JSON payload above. Read with `docker inspect` (or `skopeo inspect` / `crane config` against a registry, without pulling).
- **Secondary**: the fixed path `/opt/autoware/manifest.json` inside the image.

The operator / CI entry point (a `deploy_check.sh` shipped by the meta-repo, out of scope for this package) resolves the image set from the deploy config, extracts each image's label, writes each to a file, and invokes this package's CLI:

```bash
ros2 run autoware_component_interface_admission manifest_admit \
  manifest_0.json manifest_1.json ...
```

### `manifest_admit` exit-code contract

`manifest_admit <manifest.json> [...]` reads N per-component manifests, runs `evaluate_deploy()`, and prints one verdict line per pairing:

```text
/consumer <- /provider [/perception/object_recognition/objects]: MAJOR mismatch (code=1)
```

| Exit code | Meaning                                                                    |
| --------- | -------------------------------------------------------------------------- |
| `0`       | every pairing `ACCEPTED`                                                   |
| `1`       | at least one rejection (`MAJOR` / `MINOR` mismatch or `NO_PROVIDER`)       |
| `2`       | operational / parse error (bad usage, unreadable file, malformed manifest) |

The deploy trigger is stage 1 only, so `TOPIC_MISMATCH` is never an exit-`1` cause here — it is a runtime-only verdict (see below).

A non-zero exit blocks the deploy / OTA assembly before `docker compose up`. The gate assumes **cooperative (honest) manifests**; tamper resistance (signing / attestation) is out of scope.

## Deferred work: runtime broadcast and admission checker

The runtime broadcast of the manifest over `transient_local` and the runtime admission checker are **not implemented by this package**; they are left for a follow-up change, since the deploy-time gate is being shipped first. The runtime home of the handshake message — whether it becomes a rosidl message type — is still undecided, which is why the records here are plain C++ structs rather than rosidl messages: the field sets mirror the intended message set 1:1 so the later binding is mechanical. The shared `evaluate()` in this package is what a future runtime checker will reuse.

## Note on non-container deployments

The OCI-label deploy-time gate presupposes **multi-container images**; native / monolithic deployments are not covered by the image-label mechanism and must instead rely on the runtime trigger once it exists. That also makes packaging a component as its own container image worth doing on its own merits, independent of any other containerization motivation: it is what makes the pre-boot compatibility check available at all, a benefit a monolithic deployment only gets once the runtime trigger exists.

## License

Apache License 2.0.
