^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_interface_spec_lint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_interface_spec_lint): ratchet interface gates to error and add manifest audits (`#1315 <https://github.com/autowarefoundation/autoware_core/issues/1315>`_)
  * feat(autoware_interface_spec_lint): ratchet interface gates to error and add manifest audits
  Take the WARN-only lint landed earlier and give it teeth at the CI
  boundary: findings now fail the build instead of only being printed.
  Severity mechanism:
  - Add a Severity enum (off/warn/error) and exit_code_for(): a gate
  fails only at error severity with at least one finding.
  - Add a committed gate config (config/interface_gates.yaml) mapping each
  gate to a severity, loaded via --config with a packaged default path.
  The loader rejects unknown gate names and rejects enabling a deferred
  gate. --warn-only remains as an explicit local/advisory override.
  New manifest audits (adapted to the committed schema; multi-manifest
  capable so a future vendor-partition manifest can be cross-checked):
  - owner_isolation(manifests, base_owner): a non-base-owner entry whose
  interface name shadows a base-owner entry is a finding (vacuously clean
  on the single core manifest).
  - no_raw_spec_topic(manifests, deny, allow): a versioned topic whose name
  contains a heavy-raw deny substring is a finding; request/response
  services are out of scope (the sanctioned differential-map query
  service is not flagged), and the derived grid obstacle_grid does not
  match.
  Ratchet:
  - interface_spec_concept, spec_registered, version_consistency,
  qos_consistency, manifest_fresh, owner_isolation and no_raw_spec_topic
  are set to error.
  - The pre-commit hook drops --warn-only, so a finding now fails it.
  no_raw_spec_topic's heavy-raw deny list still lists the retired
  point_cloud_map topic name; the committed manifest no longer carries
  that entry, so the check reports no findings for it.
  manifest_fresh stays advisory locally (skips without a generator) and
  leans on the specs package's own committed-manifest gtest and CMake
  compile definition -- both already in place on this branch's base -- for
  the hard drift gate in build-and-test CI.
  Honest deferrals: no_foreign_if_dependency / profile_compat (need
  per-component role manifests / a vendor-specific interface profile that
  does not exist yet) and the runtime gates if_usage_coverage /
  admission_smoke (need runtime introspection data this static-analysis
  package does not have) are kept off with documented reasons. No
  universe-side gate wiring yet: the universe versioned surface is
  exactly the core re-exports (single version authority in core) and
  universe-owned specs are unversioned until a future vendor-partition
  manifest lands.
  * fix(autoware_interface_spec_lint): drop the 0.x-only major-version policy
  version_consistency runs at error severity once the gates ratchet, so
  the hard-coded 'MAJOR must be 0' finding would fail CI on the first
  legitimate MAJOR bump by construction. Keep the load-bearing half of
  the check (header-vs-manifest version agreement) and leave bump
  discipline to review.
  * fix(autoware_interface_spec_lint): reject empty gate configs and cover run()'s exit code
  An empty (or absent) `gates:` key loaded successfully with every gate
  defaulting to off, so the lint would report zero findings and exit 0
  against any tree regardless of its actual state. load_config() now
  raises ValueError when the parsed severities map is empty, matching
  the fail-closed contract the loader already applies to unknown gate
  names and deferred-gate misuse.
  test_config.py also pins qos_consistency into the set of gates the
  committed config ratchets to error severity; the existing assertion
  listed every implemented gate except that one.
  No prior test drove run() itself over a violating tree with an
  error-severity gate enabled: test_severity.py covers exit_code_for in
  isolation, and test_acceptance.py only pins the converse (a clean tree
  exits 0). Add test_run_exit_code.py to close that gap and assert
  run() returns 1 on an error-severity finding, stays at 0 under
  warn_only, and stays at 0 when the only finding is warn severity.
  * test(autoware_interface_spec_lint): drop the duplicate out-of-scope service test
  test_differential_pcd_map_service_stays_out_of_scope fed the identical manifest
  entry to the identical call and asserted the identical result as
  test_service_with_denied_substring_is_out_of_scope, so it added no coverage.
  Its comment also misdescribed what it guarded: the deny entry it was added
  alongside, "point_cloud_map", is not a substring of
  /map/get_differential_pointcloud_map at all -- the pre-existing "pointcloud_map"
  entry is the one that matches. Since no_raw_spec_topic scopes on
  kind == "topic" and never on the name, one service case covers every deny
  spelling, so fold that reasoning into the surviving test's comment.
  Addresses a review comment on `#1315 <https://github.com/autowarefoundation/autoware_core/issues/1315>`_.
  ---------
* feat(autoware_interface_spec_lint): add WARN-only interface spec CI lint (`#1260 <https://github.com/autowarefoundation/autoware_core/issues/1260>`_)
  * feat(autoware_interface_spec_lint): add WARN-only interface spec CI lint
  Add an ament_python lint package with four advisory checks over the
  component interface specs. All checks are WARN-only in M0: the tool prints
  findings but always exits 0 with --warn-only, so nothing fails yet. The
  warn->error ratchet is a later milestone (M2).
  - checks.py: a shared header parser plus interface_spec_concept,
  spec_registered, version_consistency (pure-Python static analyses over the
  domain headers) and manifest_fresh (rebuilds the M0.1 generator and diffs
  the committed interface_manifest.json).
  - spec_registered honors a fixed suppression marker,
  '// interface-spec-lint: not-versioned', on a struct's own line or the line
  above it, exempting that struct from the check.
  - manifest_fresh locates the generator via --generator or the
  INTERFACE_MANIFEST_GENERATOR env var, records drift as a WARN in M0, and
  skips gracefully when the generator or manifest is unavailable. M2 flips it
  to a hard failure.
  - main.py exposes the console entry ament_interface_spec_lint with --warn-only,
  --spec-dir, --manifest and --generator; scripts/run_lint.py is a repo-local
  launcher used by pre-commit so the hook works from a plain checkout.
  - .pre-commit-config.yaml gains a local 'interface-spec-lint' hook running the
  three static checks at WARN over the core specs headers (verbose, exit 0).
  - pytest tests cover each check, the suppression contract, and the
  manifest_fresh skip / drift / fresh paths.
  * feat(autoware_interface_spec_lint): teach the lint the domain macro and cross-check QoS
  Reflects the fix made for the review on the per-domain versioning
  foundation PR ("record QoS in the manifest and declare domains with a
  macro") into the lint, and rebases this branch onto that commit.
  That change moved each domain's `version` and `Specs` declarations into
  AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN and recorded every
  interface's QoS in interface_manifest.json. The lint parsed only the
  literal form, so on the rebased tree it emitted 29 false positives --
  every spec struct "unregistered", every domain declaring zero versions --
  and, because version_consistency skips a domain that does not declare
  exactly one version, its manifest-vs-header cross-check stopped running
  at all. The test suite stayed green through all of it: every fixture is
  hand-written in the old syntax, and nothing read the committed headers.
  - parse_header understands both declaration forms, including the
  multi-line invocation clang-format emits.
  - test_committed_specs.py runs every static check against the real
  committed headers and manifest. This is the gate that was missing; it
  fails on exactly the breakage a fixture-only suite cannot see.
  - qos_consistency cross-checks each registered spec's history, depth,
  reliability and durability against its manifest `qos` block.
  manifest_fresh covers the same ground, but only where the generator
  binary is built, which is not the pre-commit path -- and reliability
  and durability are the two axes ROS 2 evaluates before it lets a
  publisher and a subscription talk at all. Service specs derive their
  QoS from the one `service_qos` profile in utils.hpp rather than
  restating it, so there is no second copy to drift.
  - test_manifest_fresh asserted `all(f.level == "WARN" for f in findings)`,
  which holds on an empty list and holds again on a stale manifest, so it
  passed either way -- the same toothlessness just removed from
  test_manifest.cpp. It now asserts the committed manifest is up to date.
  - test_skips_when_generator_unavailable never cleared
  INTERFACE_MANIFEST_GENERATOR, so wherever the generator was actually
  built it ran that generator against a stub manifest and reported drift.
  It now clears the variable.
  Verified locally by building generate_interface_manifest against ROS 2
  Jazzy: the committed manifest is byte-identical to its output, 40 tests
  pass with the generator present and 39 pass with 1 skipped without it,
  the lint reports 0 warnings on the committed tree, and flipping a single
  durability in the manifest makes both the generator-backed and the
  generator-free gate fail.
  * style(autoware_interface_spec_lint): ignore the pytest delenv API in the spell checker
  spell-check-differential flagged `monkeypatch.delenv`. The repo marks such
  API names inline rather than in the shared dictionary; autoware_agnocast_wrapper's
  test_discovery_agent_launch.py already carries the identical directive for the
  same pytest call.
  * fix(autoware_interface_spec_lint): address PR review on maintainer email and entry point name
  Two inline review comments on this PR:
  - Use the tier4 maintainer address for work owned by the autowarefoundation
  or tier4 organizations, matching the 10 other package.xml files in this
  repository. Applied to package.xml and setup.py.
  - Name the console script `ament_autoware_interface_spec_lint`: the lint is
  Autoware-specific, so it carries the `autoware` prefix like the package
  itself. Renamed in setup.py's entry point, main.py's docstring and argparse
  `prog`, and the README usage examples. The pre-commit hook invokes
  scripts/run_lint.py directly and is unaffected.
  Verified: pre-commit clean, 40 tests pass, the entry point still resolves and
  reports the new prog name, and the lint reports 0 warnings on the committed
  specs.
  * test(autoware_interface_spec_lint): derive the committed-specs header set from the checks
  test_committed_specs.py re-derived the domain-header list with its own glob
  and skip set. It now calls the module's `_domain_headers`, so the headers the
  test asserts on are exactly the ones the checks scan and the two cannot drift.
  Confirmed the test still fails when the DEFINE_DOMAIN regex is stubbed out.
  * test(autoware_interface_spec_lint): run manifest_fresh against the real generator in CI
  The manifest_fresh freshness check needs the built generate_interface_manifest
  binary and otherwise skips. Locally a developer exports
  INTERFACE_MANIFEST_GENERATOR; in CI nothing did, so test_committed_manifest_is_up_to_date
  skipped on the runner. Byte-level freshness is already a hard failure in the
  specs package's own C++ test, but the Python guard was dead weight in CI.
  Wire it up:
  - test_depend on autoware_component_interface_specs. Its generator is
  BUILD_TESTING-gated, so a test-time dependency is what pulls the package into
  this one's colcon build+test and produces the installed binary. (CI already
  builds that package with BUILD_TESTING on -- PR `#90 <https://github.com/autowarefoundation/autoware_core/issues/90>`_'s test_manifest.cpp
  depends on the same target.)
  - test/conftest.py discovers the installed binary through the ament index
  (<prefix>/lib/<pkg>/generate_interface_manifest, the ament_auto_package
  default) and exports INTERFACE_MANIFEST_GENERATOR before the tests run. An
  explicit env var still wins, so local hand-built runs are unchanged, and the
  check degrades to its existing graceful skip when the package is absent or
  ament is not importable (bare pre-commit).
  Also fix the README suppression example, which used PointCloudMap -- now
  registered in map.hpp's DEFINE_DOMAIN, so it was no longer an un-versioned
  interface. Replaced with an illustrative debug/visualization topic and noted
  that no committed header actually carries the marker.
  Verified against a real generator built for ROS 2 Jazzy and a mirrored ament
  install layout: with no env var set the conftest discovers the binary and
  test_committed_manifest_is_up_to_date runs and passes (40 passed, 0 skipped);
  with the package absent or ament unavailable it skips (39 passed, 1 skipped),
  never errors. pre-commit clean; the lint still reports 0 warnings on the
  committed specs.
  * style(autoware_interface_spec_lint): reword package.xml comment to satisfy the spell checker
  spell-check-differential flagged `conftest` in the test_depend comment. Reword
  to describe the pytest setup without the jargon word; the mechanism is unchanged.
  * chore(autoware_interface_spec_lint): describe the advisory phase without internal milestone labels
  The package text referred to internal rollout milestones (M0/M0.1/M2) that have
  no meaning outside the campaign. Reword the README, module docstrings, comments,
  and the WARN summary line to describe the same advisory-now / error-later
  behavior in self-contained terms. No behavior change.
  ---------
* Contributors: Yutaka Kondo, github-actions
