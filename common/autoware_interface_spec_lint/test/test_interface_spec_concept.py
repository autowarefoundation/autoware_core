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
import textwrap

from autoware_interface_spec_lint.checks import interface_spec_concept


def _write(tmp_path: Path, body: str) -> Path:
    d = tmp_path / "component_interface_specs"
    d.mkdir(parents=True)
    f = d / "localization.hpp"
    f.write_text(textwrap.dedent(body))
    return d


def test_valid_topic_and_service_pass(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
          static constexpr size_t depth = 1;
          static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
          static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        };
        struct Initialize {
          using Service = autoware_localization_msgs::srv::InitializeLocalization;
          static constexpr char name[] = "/localization/initialize";
        };
        }
        """,
    )
    assert interface_spec_concept(spec_dir, None) == []


def test_incomplete_topic_warns(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
          static constexpr size_t depth = 1;
        };
        }
        """,
    )
    findings = interface_spec_concept(spec_dir, None)
    assert len(findings) == 1
    assert "KinematicState" in findings[0].message
    assert findings[0].level == "WARN"


def test_nonconforming_struct_with_trailing_comment_warns(tmp_path):
    # Same trailing-comment declaration form: the struct must still be scanned,
    # so its missing `durability` is reported.
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState  // canonical producer name
        {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
          static constexpr size_t depth = 1;
          static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        };
        }
        """,
    )
    findings = interface_spec_concept(spec_dir, None)
    assert len(findings) == 1
    assert "KinematicState" in findings[0].message
