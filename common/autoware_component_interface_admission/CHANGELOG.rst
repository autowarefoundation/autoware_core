^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_component_interface_admission
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* chore: align package versions to 1.9.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_component_interface_admission): deploy-time interface admission with spec QoS conformance (`#1317 <https://github.com/autowarefoundation/autoware_core/issues/1317>`_)
  * feat(autoware_component_interface_admission): add the interface admission gate package
  Reimplementation target of the universe proof of concept, now
  core-resident so the deploy-time admission gate builds against the
  released core without depending on autoware_component_interface_specs.
  The universe fork pull request is superseded by this port.
  * feat(autoware_component_interface_admission): parse QoS-carrying v2 manifests and the spec QoS table
  A v2 manifest entry may carry a `qos` block (reliability / durability /
  depth) and may omit its version fields entirely, so has_qos and has_version
  now record what the source document actually declared instead of letting an
  absent field default silently. The version fields stay an all-or-none group:
  a partial declaration is a malformed document and throws.
  spec_qos_from_json() parses autoware_component_interface_specs'
  interface_manifest.json into the interface_name -> QosRecord table the
  deploy gate holds every endpoint to. It requires a top-level `interfaces`
  array whose every entry carries both `interface` and `qos`: accepting an
  unrelated document would hand back an empty table, which would disable the
  conformance check for every endpoint without saying so.
  * feat(autoware_component_interface_admission): require every endpoint to use the QoS its spec declares
  A specification that declares RELIABLE means the interface is carried
  without drops or reordering. A subscription that quietly requests
  BEST_EFFORT still connects under DDS's request-vs-offered rule but no longer
  gets that property, and preventing exactly that class of mistake is what
  declaring the QoS in the specification is for. So the declared reliability
  and durability are an exact requirement for both sides: an endpoint that
  deviates -- in either direction, since offering TRANSIENT_LOCAL where the
  spec says VOLATILE is still not what consumers were told to expect -- is
  reported as QOS_SPEC_MISMATCH.
  The check is per endpoint, not per pairing: conformance is a property of a
  single endpoint and its spec, so a publisher-only image with no consumer
  anywhere in the deploy set, and a second provider a stage-1 match never
  picked, are each exactly as checkable as a matched pair.
  For an interface the spec set declares nothing about (a vendor or
  out-of-tree interface) there is nothing to hold either side to, so the gate
  falls back to a direct offered-vs-requested DDS check on the one
  stage-1-matched pair (QOS_PAIR_INCOMPATIBLE). That catches a pairing which
  cannot connect at all; it does not pretend to enforce a specification that
  does not exist. depth is endpoint-local and never enters a verdict, and an
  out-of-vocabulary policy string matches nothing and ranks incomparable, so
  both paths fail closed.
  * feat(autoware_component_interface_admission): add manifest_admit's --spec-manifest option and array-root fragments
  manifest_admit gains an optional `--spec-manifest <interface_manifest.json>`
  (repeatable; the last occurrence wins), which loads the spec QoS table so
  every provided / required entry carrying `qos` is held to what its
  specification declares. Omitting it writes a warning to stderr rather than
  quietly skipping the check: for a safety-adjacent gate, a deploy whose
  endpoints deviate from their specs must never exit 0 in silence.
  Each positional file may now hold either a single manifest document or a
  JSON array of them, which is the on-disk shape of a multi-node package's
  installed fragment. A malformed element inside an array fails closed exactly
  like a malformed single-document file.
  main() moves into run_manifest_admit() so the argument parsing and the
  exit-code contract are exercised directly by unit tests instead of by
  spawning a process; manifest_admit.cpp is now a thin argv/stdout/stderr
  wrapper.
  ---------
* Contributors: Yutaka Kondo, github-actions
