# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Console entry point for the interface spec lint (``ament_autoware_interface_spec_lint``).

The severity of each gate comes from the committed gate config
(``config/interface_gates.yaml``); every implemented gate is set to ``error``
so a finding fails the build. ``--warn-only`` overrides every gate to exit 0
for local advisory runs.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from autoware_interface_spec_lint.checks import interface_spec_concept
from autoware_interface_spec_lint.checks import load_manifests
from autoware_interface_spec_lint.checks import manifest_fresh
from autoware_interface_spec_lint.checks import no_raw_spec_topic
from autoware_interface_spec_lint.checks import owner_isolation
from autoware_interface_spec_lint.checks import qos_consistency
from autoware_interface_spec_lint.checks import spec_registered
from autoware_interface_spec_lint.checks import version_consistency
from autoware_interface_spec_lint.config import load_config
from autoware_interface_spec_lint.severity import Severity
from autoware_interface_spec_lint.severity import exit_code_for

_PKG_DIR = Path(__file__).resolve().parent

# Default core specs include dir and committed manifest, relative to the repo root
# (the working directory pre-commit runs the hook from).
DEFAULT_SPEC_DIRS = (
    "common/autoware_component_interface_specs/include/autoware/component_interface_specs",
)
DEFAULT_MANIFEST = "common/autoware_component_interface_specs/interface_manifest.json"

# Candidate default gate-config paths: the config shipped in the package source
# tree (cwd-independent, works under symlink-install and pre-commit), then the
# repo-relative path (a plain checkout run from the repo root).
_DEFAULT_CONFIG_CANDIDATES = (
    _PKG_DIR.parent / "config" / "interface_gates.yaml",
    Path("common/autoware_interface_spec_lint/config/interface_gates.yaml"),
)


def _default_config() -> Path | None:
    for candidate in _DEFAULT_CONFIG_CANDIDATES:
        if candidate.is_file():
            return candidate
    return None


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ament_autoware_interface_spec_lint",
        description=(
            "Config-driven lint for the Autoware component interface specs. Severity "
            "per gate comes from --config; --warn-only forces exit 0."
        ),
    )
    parser.add_argument(
        "--spec-dir",
        action="append",
        default=None,
        help="Directory of domain headers to lint (repeatable). "
        "Defaults to the core component_interface_specs include dir.",
    )
    parser.add_argument(
        "--manifest",
        default=None,
        help="Committed interface_manifest.json used by the manifest gates.",
    )
    parser.add_argument(
        "--generator",
        default=None,
        help="Path to the manifest generator binary; enables the manifest_fresh check.",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to the gate config (interface_gates.yaml). Defaults to the packaged config.",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Always exit 0 (advisory). Without it, error-severity findings fail the run.",
    )
    return parser


def _run_gate(name, gate_config, ctx):
    """Return the list of findings for gate ``name`` given its runtime context."""
    if name in (
        "interface_spec_concept",
        "spec_registered",
        "version_consistency",
        "qos_consistency",
    ):
        fn = {
            "interface_spec_concept": interface_spec_concept,
            "spec_registered": spec_registered,
            "version_consistency": version_consistency,
            "qos_consistency": qos_consistency,
        }[name]
        findings = []
        for spec_dir in ctx["spec_dirs"]:
            findings.extend(fn(Path(spec_dir), ctx["manifest"]))
        return findings
    if name == "owner_isolation":
        return owner_isolation(ctx["manifests"], gate_config.owner_isolation_base_owner)
    if name == "no_raw_spec_topic":
        return no_raw_spec_topic(
            ctx["manifests"],
            gate_config.no_raw_spec_topic_deny,
            gate_config.no_raw_spec_topic_allow,
        )
    if name == "manifest_fresh":
        return manifest_fresh(ctx["manifest"], ctx["generator"])
    raise ValueError(f"no runner for gate '{name}'")


# The pure-Python static header checks (each takes spec_dir + manifest and returns
# findings), enumerated for real-data acceptance testing. manifest_fresh needs the
# generator and the manifest audits take a manifest list, so neither is a static check.
STATIC_CHECKS = (interface_spec_concept, spec_registered, version_consistency, qos_consistency)


# Deterministic gate run order.
_GATE_ORDER = (
    "interface_spec_concept",
    "spec_registered",
    "version_consistency",
    "qos_consistency",
    "owner_isolation",
    "no_raw_spec_topic",
    "manifest_fresh",
)


def run(spec_dirs, manifest, generator, gate_config, warn_only) -> int:
    """Run every non-off gate, print findings with their severity, return exit code."""
    valid_spec_dirs = []
    for spec_dir in spec_dirs:
        if Path(spec_dir).is_dir():
            valid_spec_dirs.append(spec_dir)
        else:
            print(f"WARN spec dir not found: {spec_dir}", file=sys.stderr)

    manifests = load_manifests([manifest]) if manifest else []
    ctx = {
        "spec_dirs": valid_spec_dirs,
        "manifest": manifest,
        "manifests": manifests,
        "generator": generator,
    }

    exit_code = 0
    total_findings = 0
    error_findings = 0
    for name in _GATE_ORDER:
        severity = gate_config.severity(name)
        if severity is Severity.OFF:
            continue
        findings = _run_gate(name, gate_config, ctx)
        for finding in findings:
            print(
                f"{severity.value.upper()} [{name}] {finding.file}:{finding.line} {finding.message}"
            )
        total_findings += len(findings)
        if severity is Severity.ERROR:
            error_findings += len(findings)
        exit_code = max(exit_code, exit_code_for(findings, severity))

    off_gates = sorted(g for g in gate_config.severities if gate_config.severity(g) is Severity.OFF)
    if off_gates:
        print(f"interface-spec-lint: gates off (deferred): {', '.join(off_gates)}")
    print(
        f"interface-spec-lint: {total_findings} finding(s), " f"{error_findings} at error severity"
    )

    if warn_only:
        if exit_code:
            print("interface-spec-lint: --warn-only set; exiting 0 despite error findings")
        return 0
    return exit_code


def main(argv=None) -> int:
    """Parse arguments and run the interface spec lint."""
    args = _build_parser().parse_args(argv)
    spec_dirs = args.spec_dir or [d for d in DEFAULT_SPEC_DIRS if Path(d).is_dir()]
    if args.manifest is not None:
        manifest = args.manifest
    elif Path(DEFAULT_MANIFEST).is_file():
        manifest = DEFAULT_MANIFEST
    else:
        manifest = None

    config_path = args.config or _default_config()
    if config_path is None:
        print("ERROR gate config not found; pass --config", file=sys.stderr)
        return 2
    try:
        gate_config = load_config(config_path)
    except (ValueError, OSError) as exc:
        print(f"ERROR invalid gate config {config_path}: {exc}", file=sys.stderr)
        return 2

    return run(spec_dirs, manifest, args.generator, gate_config, args.warn_only)


if __name__ == "__main__":
    sys.exit(main())
