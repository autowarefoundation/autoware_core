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

        }

    }

}