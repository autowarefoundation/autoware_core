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

"""End-to-end acceptance: the ratcheted gates must pass on the real tree.

Runs every error-severity gate (except the generator-backed manifest_fresh, which
skips without a generator) over the committed core headers + manifest + gate config
and asserts a zero exit code: the ratchet must not turn the current, clean tree red.
"""

from pathlib import Path

from autoware_interface_spec_lint.config import load_config
from autoware_interface_spec_lint.main import run
import pytest

_LINT_PKG = Path(__file__).resolve().parents[1]
_COMMON = _LINT_PKG.parent
_SPECS_PKG = _COMMON / "autoware_component_interface_specs"
_SPEC_DIR = _SPECS_PKG / "include" / "autoware" / "component_interface_specs"
_MANIFEST = _SPECS_PKG / "interface_manifest.json"
_CONFIG = _LINT_PKG / "config" / "interface_gates.yaml"


def test_error_severity_gates_pass_on_the_real_tree():
    if not (_SPEC_DIR.is_dir() and _MANIFEST.is_file() and _CONFIG.is_file()):
        pytest.skip("core specs package not colocated in this layout")
    gate_config = load_config(_CONFIG)
    # generator=None -> manifest_fresh skips (its binding gate is the specs gtest).
    exit_code = run([str(_SPEC_DIR)], str(_MANIFEST), None, gate_config, warn_only=False)
    assert exit_code == 0
