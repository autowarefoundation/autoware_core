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

import importlib.util
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
import launch
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


@launch_testing.markers.keep_alive
def generate_test_description():
    path = get_package_share_directory("autoware_default_adapi") + "/launch/default_adapi.launch.py"
    specification = importlib.util.spec_from_file_location("launch_script", path)
    launch_script = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(launch_script)
    return launch.LaunchDescription(
        [
            *launch_script.generate_launch_description().describe_sub_entities(),
            Node(
                package="autoware_command_gate",
                executable="autoware_command_gate_exe",
                name="autoware_command_gate",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestOperationMode(unittest.TestCase):
    def test_api_services_and_state(self):
        rclpy.init()
        node = rclpy.create_node("operation_mode_api_test")
        try:
            states = []
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(
                OperationModeState, "/api/operation_mode/state", states.append, qos
            )

            def wait_state(mode, control):
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if (
                        states
                        and states[-1].mode == mode
                        and states[-1].is_autoware_control_enabled == control
                    ):
                        return
                self.fail(
                    f"Missing API state: mode={mode}, control={control}, "
                    f"last={states[-1] if states else None}"
                )

            wait_state(OperationModeState.STOP, False)
            requests = [
                ("change_to_autonomous", OperationModeState.AUTONOMOUS, False),
                ("enable_autoware_control", OperationModeState.AUTONOMOUS, True),
                ("disable_autoware_control", OperationModeState.AUTONOMOUS, False),
                ("change_to_local", OperationModeState.LOCAL, False),
                ("change_to_remote", OperationModeState.REMOTE, False),
                ("change_to_stop", OperationModeState.STOP, False),
            ]
            for service, mode, control in requests:
                client = node.create_client(ChangeOperationMode, "/api/operation_mode/" + service)
                self.assertTrue(client.wait_for_service(timeout_sec=10), service)
                future = client.call_async(ChangeOperationMode.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                self.assertTrue(future.done(), service)
                self.assertTrue(future.result().status.success, service)
                wait_state(mode, control)
        finally:
            node.destroy_node()
            rclpy.shutdown()
