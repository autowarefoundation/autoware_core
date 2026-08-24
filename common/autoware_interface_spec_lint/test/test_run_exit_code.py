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

"""End-to-end: an error-severity gate's finding must fail ``run()``.

``test_severity.py`` unit-tests ``exit_code_for`` in isolation, and
``test_acceptance.py`` pins the converse -- a clean tree exits 0 -- but nothing
before this drove ``run()`` itself over a violating tree. That is the actual claim
this package makes: a finding now fails the build. This closes the gap between the
``exit_code_for`` unit and ``run()``'s own bookkeeping (per-gate dispatch, the
total/error finding counts, the ``max()`` reduction across gates).
"""

from pathlib import Path
import textwrap

from autoware_interface_spec_lint.config import GateConfig
from autoware_interface_spec_lint.main import run
from autoware_interface_spec_lint.severity import Severity


def _write_violating_spec_dir(tmp_path: Path) -> Path:
    # Missing `depth`/`reliability`/`durability`: neither a valid topic nor a valid
    # service, so `interface_spec_concept` reports exactly one finding on it.
    spec_dir = tmp_path / "component_interface_specs"
    spec_dir.mkdir(parents=True)
    (spec_dir / "localization.hpp").write_text(textwrap.dedent("""
            namespace autoware::component_interface_specs::localization {
            struct KinematicState {
              using Message = nav_msgs::msg::Odometry;
              static constexpr char name[] = "/localization/kinematic_state";
            };
            }
            """))
    return spec_dir


def test_run_fails_when_an_error_severity_gate_reports_a_finding(tmp_path):
    spec_dir = _write_violating_spec_dir(tmp_path)
    gate_config = GateConfig(severities={"interface_spec_concept": Severity.ERROR})
    assert run([str(spec_dir)], None, None, gate_config, warn_only=False) == 1


def test_run_stays_at_zero_under_warn_only_despite_the_same_finding(tmp_path):
    spec_dir = _write_violating_spec_dir(tmp_path)
    gate_config = GateConfig(severities={"interface_spec_concept": Severity.ERROR})
    assert run([str(spec_dir)], None, None, gate_config, warn_only=True) == 0


def test_run_passes_when_the_only_finding_is_warn_severity(tmp_path):
    spec_dir = _write_violating_spec_dir(tmp_path)
    gate_config = GateConfig(severities={"interface_spec_concept": Severity.WARN})
    assert run([str(spec_dir)], None, None, gate_config, warn_only=False) == 0
