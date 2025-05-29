#!/usr/bin/env python3

# Copyright 2023 The Autoware Contributors
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

import os
import unittest

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
    lanelet2_map_path = os.path.join(
        get_package_share_directory("autoware_map_loader"), "test/data/test_map.osm"
    )

    # First launch the map loader node to publish the lanelet2 map
    lanelet2_map_loader = Node(
        package="autoware_map_loader",
        executable="autoware_lanelet2_map_loader",
        parameters=[
            {
                "lanelet2_map_path": lanelet2_map_path,
                "center_line_resolution": 5.0,
                "use_waypoints": True,
                "allow_unsupported_version": True,
            }
        ],
    )

    # Then launch the visualizer node
    lanelet2_map_visualizer = Node(
        package="autoware_lanelet2_map_visualizer",
        executable="autoware_lanelet2_map_visualizer",
    )

    context = {}

    return (
        LaunchDescription(
            [
                lanelet2_map_loader,
                lanelet2_map_visualizer,
                # Start test after 2s - gives time for both nodes to finish initialization
                launch.actions.TimerAction(
                    period=2.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        context,
    )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
