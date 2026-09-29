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

from autoware_interface_spec_lint.checks import owner_isolation


def _mf(owner, *names):
    return {
        "owner": owner,
        "interfaces": [{"interface": n, "kind": "topic", "version": "0.1.0"} for n in names],
    }


def test_single_base_manifest_is_vacuously_clean():
    base = _mf("autowarefoundation", "/control/command/control_cmd", "/planning/trajectory")
    assert owner_isolation([base], base_owner="autowarefoundation") == []


def test_extension_may_not_shadow_a_base_boundary():
    base = _mf("autowarefoundation", "/control/command/control_cmd")
    vendor = _mf("tier4", "/control/command/control_cmd")
    findings = owner_isolation([base, vendor], base_owner="autowarefoundation")
    assert len(findings) == 1
    assert "/control/command/control_cmd" in findings[0].message
    assert "tier4" in findings[0].message


def test_extension_with_disjoint_names_is_clean():
    base = _mf("autowarefoundation", "/control/command/control_cmd")
    vendor = _mf("tier4", "/tier4/extra/debug")
    assert owner_isolation([base, vendor], base_owner="autowarefoundation") == []
