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

"""Console entry point for the interface spec lint (``ament_interface_spec_lint``)."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from autoware_interface_spec_lint.checks import interface_spec_concept
from autoware_interface_spec_lint.checks import manifest_fresh
from autoware_interface_spec_lint.checks import spec_registered
from autoware_interface_spec_lint.checks import version_consistency

# The three fast, pure-Python static checks wired into pre-commit.
STATIC_CHECKS = (interface_spec_concept, spec_registered, version_consistency)

# Default core specs include dir and committed manifest, relative to the repo root
# (the working directory pre-commit runs the hook from).
DEFAULT_SPEC_DIRS = (
    "common/autoware_component_interface_specs/include/autoware/component_interface_specs",
)
DEFAULT_MANIFEST = "common/autoware_component_interface_specs/interface_manifest.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ament_interface_spec_lint",
        description=(
            "WARN-only lint for the Autoware component interface specs "
            "(interface_spec_concept, spec_registered, version_consistency, manifest_fresh)."
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
        help="Committed interface_manifest.json used by version_consistency / manifest_fresh.",
    )
    parser.add_argument(
        "--generator",
        default=None,
        help="Path to the M0.1 manifest generator binary; enables the manifest_fresh check.",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Always exit 0 (M0 behavior). Without it, findings cause a non-zero exit.",
    )
    return parser


def run(spec_dirs, manifest, generator, warn_only) -> int:
    """Run the checks over the given spec dirs, print findings, return an exit code."""
    findings = []
    for spec_dir in spec_dirs:
        path = Path(spec_dir)
        if not path.is_dir():
            print(f"WARN spec dir not found: {spec_dir}", file=sys.stderr)
            continue
        for check in STATIC_CHECKS:
            findings.extend(check(path, manifest))
    if generator is not None:
        findings.extend(manifest_fresh(manifest, generator))

    for finding in findings:
        print(f"{finding.level} {finding.file}:{finding.line} {finding.message}")
    print(
        f"interface-spec-lint: {len(findings)} warning(s) "
        f"[WARN-only, M0 does not fail the build]"
    )

    if warn_only:
        return 0
    return 1 if findings else 0


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
    return run(spec_dirs, manifest, args.generator, args.warn_only)


if __name__ == "__main__":
    sys.exit(main())
