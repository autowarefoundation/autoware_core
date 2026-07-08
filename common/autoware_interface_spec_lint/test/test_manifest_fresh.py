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

import os
from pathlib import Path
import stat

from autoware_interface_spec_lint.checks import GENERATOR_ENV
from autoware_interface_spec_lint.checks import manifest_fresh
import pytest


def _make_stub_generator(tmp_path: Path, out_content: str) -> Path:
    gen = tmp_path / "generate_stub"
    gen.write_text(
        "#!/usr/bin/env python3\n"
        "import sys\n"
        "with open(sys.argv[1], 'w') as f:\n"
        f"    f.write({out_content!r})\n"
    )
    gen.chmod(gen.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
    return gen


def test_skips_when_generator_unavailable(tmp_path):
    manifest = tmp_path / "interface_manifest.json"
    manifest.write_text('{"owner": "x", "interfaces": []}\n')
    # No generator provided and env not set -> record-only skip (M0), never raises.
    assert manifest_fresh(manifest, generator=None) == []


def test_fresh_manifest_has_no_findings(tmp_path):
    content = '{"owner": "autowarefoundation", "interfaces": []}\n'
    manifest = tmp_path / "interface_manifest.json"
    manifest.write_text(content)
    gen = _make_stub_generator(tmp_path, content)
    assert manifest_fresh(manifest, generator=gen) == []


def test_drift_is_recorded_as_warning(tmp_path):
    manifest = tmp_path / "interface_manifest.json"
    manifest.write_text('{"owner": "autowarefoundation", "interfaces": []}\n')
    gen = _make_stub_generator(tmp_path, '{"owner": "autowarefoundation", "interfaces": [1]}\n')
    findings = manifest_fresh(manifest, generator=gen)
    # M0 is record-only: drift is detected and reported as WARN, but this test
    # never fails on drift itself. M2 flips this to a hard failure.
    assert len(findings) == 1
    assert findings[0].level == "WARN"
    assert "stale" in findings[0].message


def test_real_generator_against_committed_manifest():
    gen = os.environ.get(GENERATOR_ENV)
    if not gen or not Path(gen).is_file():
        pytest.skip(f"{GENERATOR_ENV} not set (generator not built in this workspace)")
    committed = (
        Path(__file__).resolve().parents[2]
        / "autoware_component_interface_specs"
        / "interface_manifest.json"
    )
    if not committed.is_file():
        pytest.skip("committed interface_manifest.json not found in this workspace")
    findings = manifest_fresh(committed, generator=gen)
    # Record-only in M0: only assert the finding shape, never fail on drift.
    assert all(f.level == "WARN" for f in findings)
