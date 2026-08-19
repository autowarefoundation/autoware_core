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

`NO_PROVIDER` is a **completeness** verdict, not a version verdict: it fires whenever a required entry — versioned or not — has no provider of its `interface_name` anywhere in the set at all, matching the pre-v2 behavior where every required entry got this check. For a required entry that DOES declare a version, it additionally fires when every provider that exists is itself unversioned (`has_version == false`), since none of them is then version-checkable; an unversioned required entry has no version bounds to check in the first place, so it is satisfied by any provider regardless of that provider's own `has_version` — two sides both declining a version claim is a coherent unversioned pairing, not a gap to reject.

A required entry declared without a version at all (`has_version == false`) never produces `MAJOR_MISMATCH` / `MINOR_MISMATCH` at either trigger, since version bounds simply do not apply. But `TOPIC_MISMATCH` is a wiring verdict, not a version one, so `has_version` does not suppress it: at the runtime trigger, an unversioned required entry is still checked against stage 2 exactly like a versioned one — a remap that leaves it on a disjoint wire topic is still caught, rather than silently accepted because no version comparison was in play.

The deploy-time gate matches on **version + `interface_name` only** (stage 1). The remap-resolved `resolved_name` match (stage 2 of the rule) is runtime-only, because remaps live in the launch / compose layer and are not visible in image metadata — so `evaluate_deploy()` never inspects `resolved_name` and never emits `TOPIC_MISMATCH`. That residual remap false-accept is exactly what the runtime trigger backstops.

### QoS verdicts (deploy-time only)

A v2 manifest entry may also carry a `qos` block (`reliability`, `durability`, `depth`; see the JSON schema below). The QoS an interface's specification declares (from `autoware_component_interface_specs`' `interface_manifest.json`, parsed by `spec_qos_from_json()`) is an **exact requirement** for both sides, not a bound either side may deviate from. A specification that says `RELIABLE` means the interface is carried without drops or reordering; a subscription that quietly requests `BEST_EFFORT` still connects under DDS's request-vs-offered rule but no longer gets that property, and preventing exactly that class of mistake is what declaring the QoS in the specification is for. Deviating in the "stronger" direction (`TRANSIENT_LOCAL` where the spec says `VOLATILE`) is rejected on the same grounds: it is still not what every consumer written against the specification was told to expect.

How that is checked depends on whether the spec set declares a QoS for the interface at all:

- **With a declared QoS**: `evaluate_deploy()` requires every `provided` entry and every `required` entry that carries `qos` to use exactly that `reliability` and `durability`, **independently of pairing** — a publisher-only image with no consumer anywhere in the set, or a second provider of the same interface that a stage-1 match never picked, is exactly as checkable as a matched pair, because conformance is a property of the single endpoint and its spec. Such a verdict names only the one side it is about (see `AdmissionResult`, below).
- **Without a declared QoS** (e.g. a vendor / out-of-tree interface): there is nothing to hold each endpoint to in isolation, so the gate falls back to a direct offered-vs-requested DDS compatibility check on the one stage-1-matched pairing, and only when **both** sides of that pairing carry `qos`. This catches a pairing that cannot connect at all; it does not, and cannot, enforce a specification that does not exist.

| Situation                                                                                                         | Verdict                 | Code | Per-endpoint or per-pair |
| ----------------------------------------------------------------------------------------------------------------- | ----------------------- | ---- | ------------------------ |
| the spec set declares a QoS for this interface and an endpoint's `qos` differs from it                            | `QOS_SPEC_MISMATCH`     | 5    | per-endpoint             |
| the spec set declares no QoS and the one stage-1-matched pairing's offered/requested QoS is directly incompatible | `QOS_PAIR_INCOMPATIBLE` | 6    | per-pair                 |

A provider-side and a consumer-side `QOS_SPEC_MISMATCH` for the same interface are reported as two separate rows (`AdmissionResult` leaves the other side's node field empty for that code), never merged into one.

`depth` is endpoint-local and **presentational only**: it never participates in a verdict, on either path above. A required entry declared without a version at all (`has_version == false` in a v2 document) never produces a version verdict, but a provider is still resolved for it by `interface_name` alone (if one exists) so the pairwise QoS fallback has a pairing to check; if none exists, it is `NO_PROVIDER` like any other required entry (see above). `reliability_rank()` / `durability_rank()` in `admission_rule.hpp` express DDS's own offered-vs-requested strength order on this package's JSON string encoding, and are used **only** by that fallback — they never relax the exact requirement above. An out-of-vocabulary policy string ranks as incomparable and matches nothing (fail closed): every comparison against it fails.

## Records and JSON schema

`records.hpp` defines plain C++ structs that mirror the (future) handshake message set field-for-field, so the eventual rosidl binding is mechanical:

- `ProvidedInterface { ns, interface_name, resolved_name, type_name, major, minor, patch, has_version, has_qos, qos }`
- `RequiredInterface { ns, interface_name, resolved_name, type_name, accept_major_min, accept_major_max, min_minor, has_version, has_qos, qos }`
- `InterfaceManifest { owner, node_name, provided[], required[] }`
- `QosRecord { reliability, durability, depth }` — `reliability` is `"reliable"` or `"best_effort"`; `durability` is `"volatile"` or `"transient_local"`.

`interface_name` is the spec-declared name (`Spec::name`), remap-invariant and the matching key; `resolved_name` is the remap-resolved fully-qualified name, equal to `interface_name` when not remapped.

`manifest_json.hpp` serializes a manifest to / parses it from this JSON payload (the OCI-label payload schema below; this is the **v1** shape, where the version fields are always present and `qos` is absent):

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

`from_json()` is **defensive**: any malformed input — a JSON syntax error, a non-object root, a missing required key, or a value of the wrong type — is reported by throwing `std::runtime_error`, never undefined behaviour or a crash. Required per entry: `interface_name`. The version fields (`major`/`minor`/`patch` for a provided entry, `accept_major_min`/`accept_major_max`/`min_minor` for a required one) are a **group**: all three present parses as versioned; all three absent parses as unversioned (`has_version = false`); any other combination is a malformed partial declaration and throws. `qos` is optional; when present it populates `has_qos = true` and the parsed `QosRecord`, and an out-of-vocabulary `reliability` / `durability` string throws. Optional-with-default: `ns` / `type_name` / `owner` / `node_name` default to `""`, `resolved_name` defaults to `interface_name`, and the `provided` / `required` arrays default to empty when absent.

A **v2** entry may look like this instead — no version fields, a `qos` block:

```json
{
  "interface_name": "/some/topic",
  "qos": { "reliability": "reliable", "durability": "volatile", "depth": 1 }
}
```

Two more entry points in `manifest_json.hpp` round out parsing:

- `manifests_from_json(doc)` parses a **document set**: an object root yields one manifest (equivalent to `from_json()`); an array root yields one manifest per element. `manifest_admit` parses every positional file with this function (see the fragment-discovery note below), so a fragment file may hold either shape.
- `spec_qos_from_json(doc)` parses `autoware_component_interface_specs`' `interface_manifest.json` into the `interface_name -> QosRecord` map `evaluate_deploy()` takes as its spec QoS table. It **requires** a top-level `interfaces` array whose every entry is an object carrying both `interface` and `qos`, and throws otherwise — accepting an unrelated document would hand back an empty table that silently disables the spec-conformance verdict for every endpoint.

## Deploy-time gate: OCI label contract

Each component image carries its interface manifest as **pure image metadata**, so the gate reads it without creating or starting a container and without any source present in the image (a binary-only third-party image works):

- **Primary**: the OCI image label `org.autoware.interface_manifest`, whose value is the JSON payload above. Read with `docker inspect` (or `skopeo inspect` / `crane config` against a registry, without pulling).
- **Fallback**: for an image with no such label, the `interface_manifest_fragment.json` file(s) installed under `/opt/autoware/share/<pkg>/` by every package that registers interfaces through `autoware_component_interface_utils` (see Fragment discovery, below). Reading this fallback still never boots the image: a container is created from it only so its filesystem can be read with `docker cp`, and it is removed again without ever being started.

The operator / CI entry point (a `deploy_check.sh` shipped by the meta-repo, out of scope for this package) resolves the image set from the deploy config, extracts each image's manifest(s) (label primary, installed fragments as fallback), writes each to a file, and invokes this package's CLI. `--spec-manifest` is optional; pass it to also load the declared QoS described above, so every `provided` / `required` entry that carries `qos` is checked against its specification (repeatable: the last occurrence wins):

```bash
ros2 run autoware_component_interface_admission manifest_admit \
  --spec-manifest interface_manifest.json \
  manifest_0.json manifest_1.json ...
```

### Fragment discovery

Each component package that registers interfaces through `autoware_component_interface_utils` installs its own manifest fragment at the fixed relative path `share/<package_name>/interface_manifest_fragment.json`. That fragment file holds either one manifest document (a single-node package) or a JSON array of them (a multi-node package) — see `autoware_component_interface_utils`' README for the source-side convention. `manifest_admit` takes one fragment file per package (matching this one-fragment-per-package layout) as its positional arguments and parses each with `manifests_from_json()`, so both shapes are accepted uniformly: an operator or CI script assembles the deploy-time file list by discovering these installed fragments directly (e.g. globbing every package's `share` directory for that filename) rather than by pre-combining them into a single document.

### `manifest_admit` exit-code contract

`manifest_admit [--spec-manifest <interface_manifest.json>] <manifest.json> [...]` reads N per-component manifests, runs `evaluate_deploy()`, and prints one verdict line per pairing:

```text
/consumer <- /provider [/perception/object_recognition/objects]: MAJOR mismatch (code=1)
```

| Exit code | Meaning                                                                                                                                                                                           |
| --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`       | every row `ACCEPTED` — including a pairing missing `qos` on one or both sides, which counts as `ACCEPTED` with a warning (see below), and including the absence of any per-endpoint spec mismatch |
| `1`       | at least one rejection (`MAJOR` / `MINOR` mismatch, `NO_PROVIDER`, or a `QOS_*` verdict)                                                                                                          |
| `2`       | operational / parse error (bad usage, unreadable file, malformed manifest or spec manifest)                                                                                                       |

The deploy trigger is stage 1 only, so `TOPIC_MISMATCH` is never an exit-`1` cause here — it is a runtime-only verdict (see below).

For every `ACCEPTED` pairing where the pairwise QoS fallback could not run because the matched provider or required entry (or both) has no `qos`, `manifest_admit` writes a non-fatal `warning: ...; QoS compatibility was not evaluated for this pairing` line to stderr. That row is still `ACCEPTED`, but this does not by itself guarantee an overall exit `0`: an endpoint elsewhere in the same deploy set can still independently reject with `QOS_SPEC_MISMATCH`.

When `--spec-manifest` is omitted entirely, `manifest_admit` writes a `warning: ...; the spec QoS conformance verdict (QOS_SPEC_MISMATCH) is disabled for this run` line to stderr, since without a spec QoS table the per-endpoint check cannot run for any interface — a deploy whose endpoints deviate from their specs' declared QoS can then exit `0`. That must never be a silent no-op for a safety-adjacent gate, which is also why `docker/tools/test/admit-tool-entrypoint.sh` echoes its own warning when it finds no spec manifest to pass.

A non-zero exit blocks the deploy / OTA assembly before `docker compose up`. The gate assumes **cooperative (honest) manifests**; tamper resistance (signing / attestation) is out of scope.

## Deferred work: runtime broadcast and admission checker

The runtime broadcast of the manifest over `transient_local` and the runtime admission checker are **not implemented by this package**; they are left for a follow-up change, since the deploy-time gate is being shipped first. The runtime home of the handshake message — whether it becomes a rosidl message type — is still undecided, which is why the records here are plain C++ structs rather than rosidl messages: the field sets mirror the intended message set 1:1 so the later binding is mechanical. The shared `evaluate()` in this package is what a future runtime checker will reuse.

## Note on non-container deployments

The OCI-label deploy-time gate presupposes **multi-container images**; native / monolithic deployments are not covered by the image-label mechanism and must instead rely on the runtime trigger once it exists. That also makes packaging a component as its own container image worth doing on its own merits, independent of any other containerization motivation: it is what makes the pre-boot compatibility check available at all, a benefit a monolithic deployment only gets once the runtime trigger exists.

## License

Apache License 2.0.
