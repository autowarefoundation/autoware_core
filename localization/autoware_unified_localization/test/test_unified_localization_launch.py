# Copyright 2025 Autoware Foundation
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
import time
import unittest

from ament_index_python import get_package_share_directory
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_localization_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
import launch_testing
import pytest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry

# Match component_interface_specs: /localization/initialization_state uses TRANSIENT_LOCAL
QOS_INITIALIZATION_STATE = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)

# Use service Request constants: AUTO=0, DIRECT=1 (autoware_localization_msgs)
logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("autoware_unified_localization")
    launch_file = os.path.join(pkg_share, "launch", "unified_localization.launch.py")
    ld = IncludeLaunchDescription(AnyLaunchDescriptionSource(launch_file))
    return launch.LaunchDescription([ld, launch_testing.actions.ReadyToTest()])


class TestUnifiedLocalizationNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")
        self.evaluation_time = 2.0

    def tearDown(self):
        self.test_node.destroy_node()

    def test_kinematic_state_published_after_initial_pose_and_measurements(self):
        # Publish initial pose to both possible topic names (node may use /initialpose or /localization_node/initialpose)
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        init_pose.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        pub_init_1 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        pub_init_2 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/localization_node/initialpose", 10
        )
        for _ in range(30):
            pub_init_1.publish(init_pose)
            pub_init_2.publish(init_pose)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)
            time.sleep(0.05)

        # Publish pose and twist to both possible topic names (remap may produce either)
        pub_pose_1 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/in_pose_with_covariance", 10
        )
        pub_pose_2 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/localization_node/in_pose_with_covariance", 10
        )
        pub_twist_1 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/in_twist_with_covariance", 10
        )
        pub_twist_2 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/localization_node/in_twist_with_covariance", 10
        )
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance = init_pose.pose.covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = 0.5
        twist_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        for _ in range(20):
            pose_msg.header.stamp = self.test_node.get_clock().now().to_msg()
            twist_msg.header.stamp = pose_msg.header.stamp
            pub_pose_1.publish(pose_msg)
            pub_pose_2.publish(pose_msg)
            pub_twist_1.publish(twist_msg)
            pub_twist_2.publish(twist_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)

        # Subscribe to kinematic_state (remap may produce /kinematic_state or /localization_node/kinematic_state)
        odom_buffer = []
        self.test_node.create_subscription(
            Odometry, "/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )
        self.test_node.create_subscription(
            Odometry, "/localization_node/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )

        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if len(odom_buffer) > 0:
                break

        self.assertGreater(len(odom_buffer), 0, "Expected at least one Odometry on /kinematic_state")

    def test_kinematic_state_published_after_initialize_service_and_measurements(self):
        # Set initial pose via /localization/initialize service (DIRECT method)
        service_name = "/localization/initialize"
        client = self.test_node.create_client(InitializeLocalization, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {service_name} not ready")
        req = InitializeLocalization.Request()
        req.method = InitializeLocalization.Request.DIRECT
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.pose.position.x = 0.0
        pose_stamped.pose.pose.position.y = 0.0
        pose_stamped.pose.pose.position.z = 0.0
        pose_stamped.pose.pose.orientation.w = 1.0
        pose_stamped.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        req.pose_with_covariance = [pose_stamped]
        future = client.call_async(req)
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if future.done():
                break
            time.sleep(0.05)
        self.assertTrue(future.done(), "Initialize service call did not complete")
        res = future.result()
        self.assertTrue(res.status.success, f"Initialize service failed: {res.status.message}")

        # Publish pose and twist so the pipeline publishes kinematic_state
        pub_pose_1 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/in_pose_with_covariance", 10
        )
        pub_pose_2 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/localization_node/in_pose_with_covariance", 10
        )
        pub_twist_1 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/in_twist_with_covariance", 10
        )
        pub_twist_2 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/localization_node/in_twist_with_covariance", 10
        )
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance = pose_stamped.pose.covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = 0.5
        twist_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        for _ in range(20):
            pose_msg.header.stamp = self.test_node.get_clock().now().to_msg()
            twist_msg.header.stamp = pose_msg.header.stamp
            pub_pose_1.publish(pose_msg)
            pub_pose_2.publish(pose_msg)
            pub_twist_1.publish(twist_msg)
            pub_twist_2.publish(twist_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)

        odom_buffer = []
        self.test_node.create_subscription(
            Odometry, "/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )
        self.test_node.create_subscription(
            Odometry, "/localization_node/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if len(odom_buffer) > 0:
                break
        self.assertGreater(
            len(odom_buffer), 0,
            "Expected at least one Odometry on /kinematic_state after Initialize service (DIRECT)",
        )

    def test_kinematic_state_published_after_initialize_service_auto_and_measurements(self):
        # Set initial pose via /localization/initialize service (AUTO method with seed pose)
        service_name = "/localization/initialize"
        client = self.test_node.create_client(InitializeLocalization, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {service_name} not ready")
        req = InitializeLocalization.Request()
        req.method = InitializeLocalization.Request.AUTO
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.pose.position.x = 1.0
        pose_stamped.pose.pose.position.y = 1.0
        pose_stamped.pose.pose.position.z = 0.0
        pose_stamped.pose.pose.orientation.w = 1.0
        pose_stamped.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        req.pose_with_covariance = [pose_stamped]
        future = client.call_async(req)
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if future.done():
                break
            time.sleep(0.05)
        self.assertTrue(future.done(), "Initialize service call did not complete")
        res = future.result()
        self.assertTrue(res.status.success, f"Initialize service failed: {res.status.message}")

        pub_pose_1 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/in_pose_with_covariance", 10
        )
        pub_pose_2 = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/localization_node/in_pose_with_covariance", 10
        )
        pub_twist_1 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/in_twist_with_covariance", 10
        )
        pub_twist_2 = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/localization_node/in_twist_with_covariance", 10
        )
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 1.0
        pose_msg.pose.pose.position.y = 1.0
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance = pose_stamped.pose.covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = 0.3
        twist_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        for _ in range(20):
            pose_msg.header.stamp = self.test_node.get_clock().now().to_msg()
            twist_msg.header.stamp = pose_msg.header.stamp
            pub_pose_1.publish(pose_msg)
            pub_pose_2.publish(pose_msg)
            pub_twist_1.publish(twist_msg)
            pub_twist_2.publish(twist_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)

        odom_buffer = []
        self.test_node.create_subscription(
            Odometry, "/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )
        self.test_node.create_subscription(
            Odometry, "/localization_node/kinematic_state", lambda msg: odom_buffer.append(msg), 10
        )
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if len(odom_buffer) > 0:
                break
        self.assertGreater(
            len(odom_buffer), 0,
            "Expected at least one Odometry on /kinematic_state after Initialize service (AUTO)",
        )

    def test_00_initialization_state_uninitialized_at_start(self):
        state_buffer = []
        for topic in ["/localization/initialization_state", "/localization_node/initialization_state"]:
            self.test_node.create_subscription(
                LocalizationInitializationState,
                topic,
                lambda msg, buf=state_buffer: buf.append(msg),
                QOS_INITIALIZATION_STATE,
            )
        # Node publishes UNINITIALIZED at 1 Hz when not initialized; wait for at least one
        end_time = time.time() + 5.0
        while time.time() < end_time and len(state_buffer) == 0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        self.assertGreater(len(state_buffer), 0, "No message on initialization_state topic")
        self.assertEqual(
            state_buffer[0].state,
            LocalizationInitializationState.UNINITIALIZED,
            "Expected UNINITIALIZED at start",
        )

    def test_initialization_state_initialized_after_initial_pose(self):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        init_pose.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        pub = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        state_buffer = []
        self.test_node.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            lambda msg: state_buffer.append(msg),
            QOS_INITIALIZATION_STATE,
        )
        self.test_node.create_subscription(
            LocalizationInitializationState,
            "/localization_node/initialization_state",
            lambda msg: state_buffer.append(msg),
            QOS_INITIALIZATION_STATE,
        )
        for _ in range(10):
            pub.publish(init_pose)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)
            time.sleep(0.05)
        end_time = time.time() + 3.0
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if any(s.state == LocalizationInitializationState.INITIALIZED for s in state_buffer):
                break
        self.assertTrue(
            any(s.state == LocalizationInitializationState.INITIALIZED for s in state_buffer),
            "Expected INITIALIZED on initialization_state after initial pose",
        )

    def test_initialization_state_initialized_after_initialize_service_direct(self):
        service_name = "/localization/initialize"
        client = self.test_node.create_client(InitializeLocalization, service_name)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Initialize service not ready")
        req = InitializeLocalization.Request()
        req.method = InitializeLocalization.Request.DIRECT
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.pose.position.x = 0.0
        pose_stamped.pose.pose.position.y = 0.0
        pose_stamped.pose.pose.orientation.w = 1.0
        pose_stamped.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        req.pose_with_covariance = [pose_stamped]
        future = client.call_async(req)
        for _ in range(50):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if future.done():
                break
            time.sleep(0.05)
        self.assertTrue(future.done())
        self.assertTrue(future.result().status.success)
        # Subscribe with TRANSIENT_LOCAL to receive last published state (INITIALIZED)
        state_buffer = []
        self.test_node.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            lambda msg: state_buffer.append(msg),
            QOS_INITIALIZATION_STATE,
        )
        self.test_node.create_subscription(
            LocalizationInitializationState,
            "/localization_node/initialization_state",
            lambda msg: state_buffer.append(msg),
            QOS_INITIALIZATION_STATE,
        )
        end_time = time.time() + 2.0
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if any(s.state == LocalizationInitializationState.INITIALIZED for s in state_buffer):
                break
        self.assertTrue(
            any(s.state == LocalizationInitializationState.INITIALIZED for s in state_buffer),
            "Expected INITIALIZED on initialization_state after Initialize (DIRECT)",
        )

    def test_diagnostics_contains_localization_node(self):
        diag_buffer = []
        self.test_node.create_subscription(
            DiagnosticArray, "/diagnostics", lambda msg: diag_buffer.append(msg), 10
        )
        end_time = time.time() + 4.0
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if diag_buffer:
                for status in diag_buffer[-1].status:
                    if getattr(status, "hardware_id", None) == "localization_node":
                        self.assertEqual(
                            status.name,
                            "localization: localization_node",
                            "Unexpected diagnostics name",
                        )
                        keys = {kv.key: kv.value for kv in status.values}
                        self.assertIn("initialized", keys)
                        self.assertIn("pose_age_sec", keys)
                        self.assertIn("twist_age_sec", keys)
                        return
        self.fail("No diagnostics status with hardware_id 'localization_node' received")

    def test_initialize_auto_empty_pose_fails_without_gnss_topic(self):
        service_name = "/localization/initialize"
        client = self.test_node.create_client(InitializeLocalization, service_name)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        req = InitializeLocalization.Request()
        req.method = InitializeLocalization.Request.AUTO
        req.pose_with_covariance = []
        future = client.call_async(req)
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if future.done():
                break
            time.sleep(0.05)
        self.assertTrue(future.done())
        res = future.result()
        self.assertFalse(
            res.status.success,
            "Initialize AUTO with empty pose should fail when GNSS topic is not set",
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
