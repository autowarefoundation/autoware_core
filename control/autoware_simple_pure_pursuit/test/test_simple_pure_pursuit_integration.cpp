// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_pure_pursuit.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>


namespace autoware::control::simple_pure_pursuit
{

    using autoware_control_msgs::msg::Control;
    using autoware_planning_msgs::msg::Trajectory;
    using autoware_planning_msgs::msg::TrajectoryPoint;
    using nav_msgs::msg::Odometry;

    namespace
    {

        /**
        * @brief Create a dummy Odometry message with some given params.
        * This dummy one is a pose on the x-y plane with a yaw angle, and a longitudinal velocity.
        *
        * @param x dummy odometry's x position.
        * @param y dummy odometry's y position.
        * @param yaw dummy odometry's yaw angle.
        * @param longitudinal_velocity dummy odometry's longitudinal velocity.
        * @param stamp dummy odometry's timestamp.
        *
        * @return Odometry dummy message.
        */
        Odometry make_odometry(
            const double x, 
            const double y, 
            const double yaw, 
            const double longitudinal_velocity,
            const builtin_interfaces::msg::Time & stamp
        ) {
            
            Odometry odom;
            odom.header.frame_id = "map";
            odom.header.stamp = stamp;
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
            odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
            odom.twist.twist.linear.x = longitudinal_velocity;

            return odom;

        };

        /**
        * @brief Create a dummy Trajectory message with some given params.
        * This dummy one is a straight line along x-axis,
        * so all y's are 0.0 and all yaw angles are 0.0.
        *
        * @param x_positions dummy trajectory's x positions.
        * @param longitudinal_velocity dummy trajectory's longitudinal velocity.
        * @param stamp dummy trajectory's timestamp.
        *
        * @return Trajectory dummy message.
        */
        Trajectory make_straight_trajectory(
            const std::vector<double> & x_positions,
            const double longitudinal_velocity,
            const builtin_interfaces::msg::Time & stamp
        ) {

            Trajectory traj;
            traj.header.frame_id = "map";
            traj.header.stamp = stamp;

            for (const auto x : x_positions) {
                TrajectoryPoint point;
                point.pose.position.x = x;
                point.pose.position.y = 0.0;
                point.longitudinal_velocity_mps = static_cast<float>(longitudinal_velocity);
                traj.points.push_back(point);
            }

            return traj;

        };

        /**
        * @brief Create a NodeOptions with some given overrides and some default config files.
        *
        * @param overrides a vector of pairs of <param name, param value> to override default ones.
        *
        * @return NodeOptions with given overrides and default config files.
        */
        rclcpp::NodeOptions make_node_options(
            const std::vector<std::pair<std::string, rclcpp::ParameterValue>> & overrides = {}
        ) {
            
            const auto autoware_test_utils_dir =
                ament_index_cpp::get_package_share_directory("autoware_test_utils");
            const auto autoware_simple_pure_pursuit_dir =
                ament_index_cpp::get_package_share_directory("autoware_simple_pure_pursuit");

            auto node_options = rclcpp::NodeOptions{};
            autoware::test_utils::updateNodeOptions(
                node_options,
                {
                    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                    autoware_simple_pure_pursuit_dir + "/config/simple_pure_pursuit.param.yaml"
                }
            );

            for (const auto & [name, value] : overrides) {
                node_options.append_parameter_override(name, value);
            }

            return node_options;

        };


        class SimplePurePursuitIntegrationHarness
        {
            
            public:

            /**
            * @brief Construct a new SimplePurePursuitIntegrationHarness object, 
                     which sets up test env for integration testing of SimplePurePursuitNode.
            *
            * @param node_options NodeOptions to construct target SimplePurePursuitNode, 
                                  can be used to override some default params if needed.
            */
            explicit SimplePurePursuitIntegrationHarness(
                const rclcpp::NodeOptions & node_options
            ) {
                
                executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

                target_node = std::make_shared<SimplePurePursuitNode>(node_options);
                input_pub_node = rclcpp::Node::make_shared("simple_pure_pursuit_test_input_publisher");
                output_sub_node = rclcpp::Node::make_shared("simple_pure_pursuit_test_output_subscriber");

                odom_pub = input_pub_node -> create_publisher<Odometry>(
                    "/simple_pure_pursuit/input/odometry", 
                    rclcpp::QoS{1}
                );
                traj_pub = input_pub_node -> create_publisher<Trajectory>(
                    "/simple_pure_pursuit/input/trajectory", 
                    rclcpp::QoS{1}
                );
                control_sub = output_sub_node -> create_subscription<Control>(
                    "/simple_pure_pursuit/output/control_command", 
                    rclcpp::QoS{1}.transient_local(),
                    [this](const Control::SharedPtr msg) {
                        {
                        std::scoped_lock lock(received_control_mutex_);
                        received_control_ = msg;
                        }
                        is_received_.store(true);
                    }
                );

                executor -> add_node(target_node);
                executor -> add_node(input_pub_node);
                executor -> add_node(output_sub_node);

                // Spin it up!
                executor_thread = std::thread([this]() { executor -> spin(); });
                (void)wait_for_connections(connection_timeout);

            };

    }; // namespace

}; // namespace autoware::control::simple_pure_pursuit