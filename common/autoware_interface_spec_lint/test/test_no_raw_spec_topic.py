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

from autoware_interface_spec_lint.checks import no_raw_spec_topic
from autoware_interface_spec_lint.config import load_config

# Bind the tests to the COMMITTED gate config rather than a hand-copied list, so a
# deny entry that cannot match the interface name it targets fails here.
_COMMITTED_CONFIG = Path(__file__).resolve().parents[1] / "config" / "interface_gates.yaml"
DENY = load_config(_COMMITTED_CONFIG).no_raw_spec_topic_deny


def _mf(*entries):
    return {"owner": "autowarefoundation", "interfaces": list(entries)}


def _topic(name):
    return {"interface": name, "kind": "topic", "version": "0.1.0"}


def _service(name):
    return {"interface": name, "kind": "service", "version": "0.1.0"}


def test_heavy_raw_topic_is_a_finding():
    mf = _mf(_topic("/sensing/lidar/concatenated/pointcloud"))
    findings = no_raw_spec_topic([mf], deny=DENY, allow=[])
    assert len(findings) == 1
    assert "concatenated/pointcloud" in findings[0].message


def test_derived_grid_topic_is_allowed():
    # /sensing/obstacle_segmentation/obstacle_grid is a derived grid, not raw; it
    # contains none of the deny substrings and must not be flagged.
    mf = _mf(_topic("/sensing/obstacle_segmentation/obstacle_grid"))
    assert no_raw_spec_topic([mf], deny=DENY, allow=[]) == []


def test_allow_list_exempts_a_denied_name():
    mf = _mf(_topic("/sensing/camera/image_raw"))
    assert no_raw_spec_topic([mf], deny=DENY, allow=["/sensing/camera/image_raw"]) == []
    # Without the allow entry it is a finding.
    assert len(no_raw_spec_topic([mf], deny=DENY, allow=[])) == 1


def test_service_with_denied_substring_is_out_of_scope():
    # /map/get_differential_pointcloud_map contains "pointcloud_map" but is a
    # request/response service, not a heavy-raw broadcast stream: not flagged.
    # The gate scopes on kind == "topic" and never on the name, so this one case
    # covers every deny spelling -- adding a substring cannot start flagging a
    # sanctioned service.
    mf = _mf(_service("/map/get_differential_pointcloud_map"))
    assert no_raw_spec_topic([mf], deny=DENY, allow=[]) == []


def test_clean_topic_is_not_flagged():
    mf = _mf(_topic("/planning/trajectory"))
    assert no_raw_spec_topic([mf], deny=DENY, allow=[]) == []


def test_committed_deny_list_catches_the_point_cloud_map_topic():
    # map.hpp declares PointCloudMap::name = "/map/point_cloud_map" (point_cloud,
    # underscored). The deny entry "pointcloud_map" is not a substring of it, so the
    # gate could never flag the one heavy-raw topic it exists to confine.
    mf = _mf(_topic("/map/point_cloud_map"))
    findings = no_raw_spec_topic([mf], deny=DENY, allow=[])
    assert len(findings) == 1
    assert "/map/point_cloud_map" in findings[0].message
