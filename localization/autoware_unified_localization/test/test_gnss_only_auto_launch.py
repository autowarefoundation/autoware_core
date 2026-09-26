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
from autoware_localization_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("autoware_unified_localization")
    launch_file = os.path.join(pkg_share, "launch", "unified_localization.launch.py")
    param_file = os.path.join(pkg_share, "config", "gnss_only_test.param.yaml")
    ld = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file),
        launch_arguments=[("param_file", param_file)],
    )
    return launch.LaunchDescription([ld, launch_testing.actions.ReadyToTest()])


class TestGnssOnlyAuto(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_gnss_only_node")
        self.evaluation_time = 2.0

    def tearDown(self):
        self.test_node.destroy_node()

    def test_gnss_only_auto_initialize_succeeds(self):
        gnss_pose = PoseWithCovarianceStamped()
        gnss_pose.header.frame_id = "map"
        gnss_pose.header.stamp = self.test_node.get_clock().now().to_msg()
        gnss_pose.pose.pose.position.x = 2.0
        gnss_pose.pose.pose.position.y = 2.0
        gnss_pose.pose.pose.position.z = 0.0
        gnss_pose.pose.pose.orientation.w = 1.0
        gnss_pose.pose.covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
        ]
        pub_gnss = self.test_node.create_publisher(PoseWithCovarianceStamped, "/test_gnss_pose", 10)
        for _ in range(15):
            gnss_pose.header.stamp = self.test_node.get_clock().now().to_msg()
            pub_gnss.publish(gnss_pose)
            rclpy.spin_once(self.test_node, timeout_sec=0.05)
            time.sleep(0.05)

        service_name = "/localization/initialize"
        client = self.test_node.create_client(InitializeLocalization, service_name)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Initialize service not ready")
        req = InitializeLocalization.Request()
        req.method = InitializeLocalization.Request.AUTO
        req.pose_with_covariance = []
        future = client.call_async(req)
        for _ in range(100):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if future.done():
                break
            time.sleep(0.05)
        self.assertTrue(future.done(), "Initialize service call did not complete")
        res = future.result()
        self.assertTrue(
            res.status.success,
            f"Initialize AUTO with empty pose (GNSS-only) should succeed: {res.status.message}",
        )

        pub_pose = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/in_pose_with_covariance", 10
        )
        pub_twist = self.test_node.create_publisher(
            TwistWithCovarianceStamped, "/in_twist_with_covariance", 10
        )
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 2.0
        pose_msg.pose.pose.position.y = 2.0
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance = gnss_pose.pose.covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = 0.5
        twist_msg.twist.covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
        ]
        for _ in range(20):
            pose_msg.header.stamp = self.test_node.get_clock().now().to_msg()
            twist_msg.header.stamp = pose_msg.header.stamp
            pub_pose.publish(pose_msg)
            pub_twist.publish(twist_msg)
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
            len(odom_buffer),
            0,
            "Expected Odometry on /kinematic_state after GNSS-only AUTO initialize",
        )


@launch_testing.post_shutdown_test()
class TestGnssOnlyProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
