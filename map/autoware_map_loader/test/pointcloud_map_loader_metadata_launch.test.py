#!/usr/bin/env python3

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

"""The map metadata must be published even when the whole-load topic is disabled.

A deployment that delivers the PCD map through the differential/partial services sets
``enable_whole_load: false``. Before this test, ``output/pointcloud_map_metadata`` was only
advertised when ``enable_selected_load`` was also true, leaving such a deployment with no
latched topic to prove the map is loaded.
"""

import os
import tempfile
import unittest

from autoware_map_msgs.msg import PointCloudMapMetaData
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

_PCD = """\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
"""


@pytest.mark.launch_test
def generate_test_description():
    tmp_dir = tempfile.mkdtemp(prefix="pcd_map_loader_test_")
    pcd_path = os.path.join(tmp_dir, "single.pcd")
    with open(pcd_path, "w") as f:
        f.write(_PCD)

    pointcloud_map_loader = Node(
        package="autoware_map_loader",
        executable="autoware_pointcloud_map_loader",
        parameters=[
            {
                "pcd_paths_or_directory": [pcd_path],
                # Absent on purpose: a single-PCD map synthesizes its metadata.
                "pcd_metadata_path": os.path.join(tmp_dir, "absent_metadata.yaml"),
                # The deployment under test: no whole-load topic, no selected-load service.
                "enable_whole_load": False,
                "enable_downsampled_whole_load": False,
                "enable_partial_load": True,
                "enable_selected_load": False,
            }
        ],
    )

    return (
        LaunchDescription(
            [
                pointcloud_map_loader,
                launch.actions.TimerAction(
                    period=1.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {},
    )


class TestMetadataIsPublished(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("pointcloud_map_loader_metadata_test")

    def tearDown(self):
        self.node.destroy_node()

    def test_metadata_is_latched_without_whole_load(self):
        received = []
        # transient_local: the loader publishes once, on construction, before we subscribe.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(
            PointCloudMapMetaData,
            "/output/pointcloud_map_metadata",
            lambda msg: received.append(msg),
            qos,
        )

        end = self.node.get_clock().now().nanoseconds + 10 * 1_000_000_000
        while not received and self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertEqual(len(received), 1, "metadata was never published")
        self.assertEqual(received[0].header.frame_id, "map")
        self.assertEqual(len(received[0].metadata_list), 1, "one PCD cell expected")

    def test_whole_load_topic_stays_unadvertised(self):
        # The metadata is the liveness signal precisely because the raw map topic is gone.
        end = self.node.get_clock().now().nanoseconds + 3 * 1_000_000_000
        while self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        names = [n for n, _ in self.node.get_topic_names_and_types()]
        self.assertNotIn("/output/pointcloud_map", names)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
