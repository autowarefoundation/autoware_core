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

from pathlib import Path

from autoware_interface_spec_lint.config import load_config
from autoware_interface_spec_lint.severity import Severity
import pytest

# The committed gate config that ships with the package.
_COMMITTED_CONFIG = Path(__file__).resolve().parents[1] / "config" / "interface_gates.yaml"


def _write(tmp_path, text):
    p = tmp_path / "interface_gates.yaml"
    p.write_text(text)
    return p


def test_off_warn_error_are_parsed(tmp_path):
    cfg = load_config(
        _write(
            tmp_path,
            "gates:\n"
            "  interface_spec_concept: { severity: error }\n"
            "  spec_registered: { severity: warn }\n"
            "  profile_compat: { severity: off }\n",
        )
    )
    assert cfg.severity("interface_spec_concept") is Severity.ERROR
    assert cfg.severity("spec_registered") is Severity.WARN
    assert cfg.severity("profile_compat") is Severity.OFF
    # A gate absent from the config defaults to off.
    assert cfg.severity("no_raw_spec_topic") is Severity.OFF


def test_unknown_gate_name_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown gate 'not_a_real_gate'"):
        load_config(_write(tmp_path, "gates:\n  not_a_real_gate: { severity: error }\n"))


def test_invalid_severity_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown severity 'fatal'"):
        load_config(_write(tmp_path, "gates:\n  spec_registered: { severity: fatal }\n"))


def test_enabling_a_deferred_gate_is_rejected(tmp_path):
    # A deferred gate has no implementation; it may only be declared off.
    with pytest.raises(ValueError, match="gate 'if_usage_coverage' is deferred"):
        load_config(_write(tmp_path, "gates:\n  if_usage_coverage: { severity: error }\n"))


def test_empty_gates_is_rejected(tmp_path):
    # An empty (or absent) 'gates:' key would otherwise load successfully with every
    # gate defaulting to off, so interface-spec-lint would report 0 findings and exit 0
    # no matter what tree it points at -- the config loader's own version of the
    # fail-open shape a missing manifest or spec dir has elsewhere in this package.
    with pytest.raises(ValueError, match="'gates' is empty"):
        load_config(_write(tmp_path, "gates: {}\n"))
    with pytest.raises(ValueError, match="'gates' is empty"):
        load_config(_write(tmp_path, "owner_isolation:\n  base_owner: acme\n"))


def test_no_raw_spec_topic_params_are_parsed(tmp_path):
    cfg = load_config(
        _write(
            tmp_path,
            "gates:\n"
            "  no_raw_spec_topic: { severity: error }\n"
            "no_raw_spec_topic:\n"
            "  deny: [/pointcloud, pointcloud_map]\n"
            "  allow: [/kept/topic]\n"
            "owner_isolation:\n"
            "  base_owner: acme\n",
        )
    )
    assert cfg.no_raw_spec_topic_deny == ["/pointcloud", "pointcloud_map"]
    assert cfg.no_raw_spec_topic_allow == ["/kept/topic"]
    assert cfg.owner_isolation_base_owner == "acme"


def test_committed_config_is_ratcheted_to_error():
    cfg = load_config(_COMMITTED_CONFIG)
    for gate in (
        "interface_spec_concept",
        "spec_registered",
        "version_consistency",
        "qos_consistency",
        "manifest_fresh",
        "owner_isolation",
        "no_raw_spec_topic",
    ):
        assert cfg.severity(gate) is Severity.ERROR, gate
    for gate in (
        "no_foreign_if_dependency",
        "profile_compat",
        "if_usage_coverage",
        "admission_smoke",
    ):
        assert cfg.severity(gate) is Severity.OFF, gate
    # Spot-check the committed heavy-raw deny list.
    assert "concatenated/pointcloud" in cfg.no_raw_spec_topic_deny
    assert cfg.no_raw_spec_topic_allow == []
