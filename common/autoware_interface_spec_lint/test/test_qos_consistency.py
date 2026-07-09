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

from autoware_interface_spec_lint.checks import qos_consistency

# The real headers declare their domain through the macro, so the fixtures do too.
_HEADER = """
namespace autoware::component_interface_specs::localization {
struct KinematicState {
  using Message = nav_msgs::msg::Odometry;
  static constexpr char name[] = "/localization/kinematic_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = %s;
};
struct Initialize {
  using Service = autoware_localization_msgs::srv::InitializeLocalization;
  static constexpr char name[] = "/localization/initialize";
};
AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0, KinematicState, Initialize)
}
"""

# The shared service profile qos_consistency derives instead of restating.
_UTILS = """
namespace autoware::component_interface_specs {
namespace service_qos {
static constexpr std::size_t depth = 10;
static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
}
}
"""

_TOPIC_QOS = {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile",
}
_SERVICE_QOS = {
    "history": "keep_last",
    "depth": 10,
    "reliability": "reliable",
    "durability": "volatile",
}


def _spec_dir(tmp_path: Path, durability="RMW_QOS_POLICY_DURABILITY_VOLATILE") -> Path:
    d = tmp_path / "component_interface_specs"
    d.mkdir(parents=True)
    (d / "localization.hpp").write_text(textwrap.dedent(_HEADER % durability))
    (d / "utils.hpp").write_text(textwrap.dedent(_UTILS))
    return d


def _manifest(tmp_path: Path, topic_qos=None, service_qos=None, drop_qos=False) -> Path:
    topic = {
        "domain": "localization",
        "interface": "/localization/kinematic_state",
        "type": "nav_msgs/msg/Odometry",
        "kind": "topic",
        "version": "0.1.0",
        "qos": dict(topic_qos or _TOPIC_QOS),
    }
    service = {
        "domain": "localization",
        "interface": "/localization/initialize",
        "type": "autoware_localization_msgs/srv/InitializeLocalization",
        "kind": "service",
        "version": "0.1.0",
        "qos": dict(service_qos or _SERVICE_QOS),
    }
    if drop_qos:
        topic.pop("qos")
    path = tmp_path / "interface_manifest.json"
    path.write_text(json.dumps({"owner": "autowarefoundation", "interfaces": [topic, service]}))
    return path


def test_matching_qos_passes(tmp_path):
    assert qos_consistency(_spec_dir(tmp_path), _manifest(tmp_path)) == []


def test_no_manifest_is_a_graceful_skip(tmp_path):
    spec_dir = _spec_dir(tmp_path)
    assert qos_consistency(spec_dir, None) == []
    assert qos_consistency(spec_dir, str(tmp_path / "no" / "such.json")) == []


def test_durability_drift_warns(tmp_path):
    # The axis a manifest-only gate would miss: a TRANSIENT_LOCAL subscription never
    # receives the latched message a VOLATILE publisher dropped.
    manifest = _manifest(tmp_path, topic_qos={**_TOPIC_QOS, "durability": "transient_local"})
    findings = qos_consistency(_spec_dir(tmp_path), manifest)
    assert len(findings) == 1
    assert findings[0].level == "WARN"
    assert "durability: specs='volatile', manifest='transient_local'" in findings[0].message


def test_reliability_drift_warns(tmp_path):
    manifest = _manifest(tmp_path, topic_qos={**_TOPIC_QOS, "reliability": "best_effort"})
    findings = qos_consistency(_spec_dir(tmp_path), manifest)
    assert len(findings) == 1
    assert "reliability: specs='reliable', manifest='best_effort'" in findings[0].message


def test_depth_drift_warns(tmp_path):
    manifest = _manifest(tmp_path, topic_qos={**_TOPIC_QOS, "depth": 5})
    findings = qos_consistency(_spec_dir(tmp_path), manifest)
    assert len(findings) == 1
    assert "depth: specs=1, manifest=5" in findings[0].message


def test_header_side_drift_warns(tmp_path):
    # The manifest is untouched; the header moved. Same finding, opposite direction.
    spec_dir = _spec_dir(tmp_path, durability="RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL")
    findings = qos_consistency(spec_dir, _manifest(tmp_path))
    assert len(findings) == 1
    assert "durability: specs='transient_local', manifest='volatile'" in findings[0].message


def test_service_qos_drift_warns(tmp_path):
    # Services carry no QoS of their own; theirs comes from utils.hpp's shared profile.
    manifest = _manifest(tmp_path, service_qos={**_SERVICE_QOS, "depth": 1})
    findings = qos_consistency(_spec_dir(tmp_path), manifest)
    assert len(findings) == 1
    assert "'Initialize'" in findings[0].message
    assert "depth: specs=10, manifest=1" in findings[0].message


def test_missing_qos_block_warns(tmp_path):
    findings = qos_consistency(_spec_dir(tmp_path), _manifest(tmp_path, drop_qos=True))
    assert len(findings) == 1
    assert "records no 'qos' block" in findings[0].message


def test_unnameable_policy_warns_instead_of_guessing(tmp_path):
    # Mirrors the generator, which throws rather than emitting a placeholder entry.
    spec_dir = _spec_dir(tmp_path, durability="RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT")
    findings = qos_consistency(spec_dir, _manifest(tmp_path))
    assert len(findings) == 1
    assert "durability policy the manifest cannot name" in findings[0].message


def test_spec_missing_from_manifest_warns(tmp_path):
    path = tmp_path / "interface_manifest.json"
    path.write_text(json.dumps({"owner": "autowarefoundation", "interfaces": []}))
    findings = qos_consistency(_spec_dir(tmp_path), path)
    assert len(findings) == 2
    assert all("the manifest has no" in f.message for f in findings)


def test_unregistered_spec_is_not_reported_twice(tmp_path):
    # A spec absent from `Specs` is spec_registered's finding. qos_consistency stays quiet
    # so one root cause does not produce two warnings.
    d = tmp_path / "component_interface_specs"
    d.mkdir(parents=True)
    (d / "localization.hpp").write_text(textwrap.dedent("""
            namespace autoware::component_interface_specs::localization {
            struct KinematicState {
              using Message = nav_msgs::msg::Odometry;
              static constexpr char name[] = "/localization/kinematic_state";
              static constexpr size_t depth = 1;
              static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
              static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            };
            AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0)
            }
            """))
    (d / "utils.hpp").write_text(textwrap.dedent(_UTILS))
    path = tmp_path / "interface_manifest.json"
    path.write_text(json.dumps({"owner": "autowarefoundation", "interfaces": []}))
    assert qos_consistency(d, path) == []
