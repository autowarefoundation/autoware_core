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

from autoware_interface_spec_lint.checks import spec_registered


def _write(tmp_path: Path, body: str) -> Path:
    d = tmp_path / "component_interface_specs"
    d.mkdir(parents=True)
    f = d / "localization.hpp"
    f.write_text(textwrap.dedent(body))
    return d


def test_registered_struct_passes(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
          static constexpr size_t depth = 1;
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;
        }
        """,
    )
    assert spec_registered(spec_dir, None) == []


def test_unregistered_struct_warns(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        struct Acceleration {
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;  // Acceleration missing
        }
        """,
    )
    findings = spec_registered(spec_dir, None)
    assert len(findings) == 1
    assert "Acceleration" in findings[0].message
    assert findings[0].level == "WARN"


def test_suppression_marker_on_line_above_exempts(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        // interface-spec-lint: not-versioned
        struct Acceleration {
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;
        }
        """,
    )
    # Acceleration is unregistered but marked not-versioned, so it is exempt.
    assert spec_registered(spec_dir, None) == []


def test_suppression_marker_on_same_line_exempts(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        struct Acceleration {  // interface-spec-lint: not-versioned
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;
        }
        """,
    )
    assert spec_registered(spec_dir, None) == []


def test_unregistered_struct_with_trailing_comment_warns(tmp_path):
    # The real domain headers declare specs as `struct Foo  // note` with the brace
    # on the next line. The struct scanner must see through the trailing comment.
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        struct Acceleration  // new spec (safety dependency)
        {
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;
        }
        """,
    )
    findings = spec_registered(spec_dir, None)
    assert len(findings) == 1
    assert "Acceleration" in findings[0].message


def test_macro_registration_is_understood(tmp_path):
    # The real headers register their specs through
    # AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN, which expands to the
    # `using Specs = std::tuple<...>` the literal fixtures above spell out. Parsing only
    # the literal form once made every macro-registered spec look unregistered.
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0, KinematicState)
        }
        """,
    )
    assert spec_registered(spec_dir, None) == []


def test_macro_registration_still_catches_an_unregistered_spec(tmp_path):
    # clang-format wraps a long invocation across lines; the scanner must see through it.
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        struct Acceleration {
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(
          0, 1, 0, KinematicState)
        }
        """,
    )
    findings = spec_registered(spec_dir, None)
    assert len(findings) == 1
    assert "Acceleration" in findings[0].message


def test_suppression_marker_survives_trailing_comment_form(tmp_path):
    spec_dir = _write(
        tmp_path,
        """
        namespace autoware::component_interface_specs::localization {
        struct KinematicState {
          using Message = nav_msgs::msg::Odometry;
          static constexpr char name[] = "/localization/kinematic_state";
        };
        struct Acceleration  // interface-spec-lint: not-versioned
        {
          using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
          static constexpr char name[] = "/localization/acceleration";
        };
        static constexpr Version version{0, 1, 0};
        using Specs = std::tuple<KinematicState>;
        }
        """,
    )
    assert spec_registered(spec_dir, None) == []
