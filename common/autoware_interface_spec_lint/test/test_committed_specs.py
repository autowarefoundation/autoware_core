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

"""The checks run clean against the real committed specs, not just against fixtures.

Every other test in this package feeds the parser a hand-written fixture, so the suite
stayed green when `autoware_component_interface_specs` moved its per-domain `version` and
`Specs` declarations into AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN: the fixtures
still used the old literal syntax, while the real headers emitted 29 false positives and
`version_consistency` stopped cross-checking the manifest entirely. A check that reads the
committed sources is the only one that notices when the sources move out from under it.
"""

from pathlib import Path

from autoware_interface_spec_lint.checks import parse_header
from autoware_interface_spec_lint.main import STATIC_CHECKS
import pytest

_SPECS_PKG = Path(__file__).resolve().parents[2] / "autoware_component_interface_specs"
_SPEC_DIR = _SPECS_PKG / "include" / "autoware" / "component_interface_specs"
_MANIFEST = _SPECS_PKG / "interface_manifest.json"

requires_committed_specs = pytest.mark.skipif(
    not _SPEC_DIR.is_dir() or not _MANIFEST.is_file(),
    reason="autoware_component_interface_specs sources not present in this workspace",
)


@requires_committed_specs
@pytest.mark.parametrize("check", STATIC_CHECKS, ids=lambda c: c.__name__)
def test_static_check_is_clean_on_the_committed_specs(check):
    findings = check(_SPEC_DIR, _MANIFEST)
    assert findings == [], "\n".join(f"{f.file}:{f.line} {f.message}" for f in findings)


@requires_committed_specs
def test_every_committed_domain_declares_a_version_and_registers_its_specs():
    # Anchors the parser to the syntax the headers actually use. Were the macro to stop
    # being understood, every domain here would report zero versions and zero members --
    # which is precisely how the check above silently lost its cross-check once before.
    domains = [p for p in _SPEC_DIR.glob("*.hpp") if p.name not in {"utils.hpp", "version.hpp"}]
    assert domains, "no domain headers found"
    for path in domains:
        if path.name == "concepts.hpp":
            continue
        header = parse_header(path)
        assert len(header.versions) == 1, f"{path.name} declares {len(header.versions)} versions"
        assert header.specs_members, f"{path.name} registers no specs"
