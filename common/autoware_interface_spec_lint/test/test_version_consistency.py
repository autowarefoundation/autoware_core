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

import json
from pathlib import Path
import textwrap

from autoware_interface_spec_lint.checks import version_consistency


def _write(tmp_path: Path, body: str) -> Path:
    d = tmp_path / "component_interface_specs"
    d.mkdir(parents=True)
    f = d / "localization.hpp"
    f.write_text(textwrap.dedent(body))
    return d


_ONE_STRUCT = """
namespace autoware::component_interface_specs::localization {
struct KinematicState {
  using Message = nav_msgs::msg::Odometry;
  static constexpr char name[] = "/localization/kinematic_state";
};
%s
using Specs = std::tuple<KinematicState>;
}
"""


def test_single_zero_x_version_passes(tmp_path):
    spec_dir = _write(tmp_path, _ONE_STRUCT % "static constexpr Version version{0, 1, 0};")
    assert version_consistency(spec_dir, None) == []


def test_missing_version_warns(tmp_path):
    spec_dir = _write(tmp_path, _ONE_STRUCT % "")
    findings = version_consistency(spec_dir, None)
    assert len(findings) == 1
    assert "exactly one" in findings[0].message
    assert findings[0].level == "WARN"


def test_non_zero_major_warns(tmp_path):
    spec_dir = _write(tmp_path, _ONE_STRUCT % "static constexpr Version version{1, 0, 0};")
    findings = version_consistency(spec_dir, None)
    assert len(findings) == 1
    assert "not 0.x" in findings[0].message


def test_manifest_version_mismatch_warns(tmp_path):
    spec_dir = _write(tmp_path, _ONE_STRUCT % "static constexpr Version version{0, 1, 0};")
    manifest = tmp_path / "interface_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "owner": "autowarefoundation",
                "interfaces": [
                    {
                        "domain": "localization",
                        "interface": "/localization/kinematic_state",
                        "type": "nav_msgs/msg/Odometry",
                        "kind": "topic",
                        "version": "0.2.0",
                    }
                ],
            }
        )
    )
    findings = version_consistency(spec_dir, manifest)
    assert len(findings) == 1
    assert "manifest version '0.2.0'" in findings[0].message
    assert "header version '0.1.0'" in findings[0].message


def test_unreadable_manifest_path_warns_and_skips(tmp_path):
    # Every other manifest-backed gate warns and skips when the manifest cannot be
    # read; version_consistency must not raise FileNotFoundError at the user.
    spec_dir = _write(tmp_path, _ONE_STRUCT % "static constexpr Version version{0, 1, 0};")
    assert version_consistency(spec_dir, str(tmp_path / "no" / "such.json")) == []
