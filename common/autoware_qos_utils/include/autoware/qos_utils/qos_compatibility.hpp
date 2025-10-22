// Copyright 2024 The Autoware Contributors
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

#ifndef AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_
#define AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_

#include <rclcpp/rclcpp.hpp>

namespace autoware::qos_utils
{

/**
 * @brief Get QoS profile for services with ROS 2 distribution compatibility
 * @return QoS profile appropriate for the current ROS 2 distribution
 * 
 * For ROS 2 Jazzy and later: returns rclcpp::ServicesQoS()
 * For earlier versions: returns rclcpp::ServicesQoS().get_rmw_qos_profile()
 */
#ifdef ROS_DISTRO_JAZZY
#define AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE() rclcpp::ServicesQoS()
#else
#define AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE() rclcpp::ServicesQoS().get_rmw_qos_profile()
#endif

/**
 * @brief Get default QoS profile with ROS 2 distribution compatibility
 * @return QoS profile appropriate for the current ROS 2 distribution
 * 
 * For ROS 2 Jazzy and later: returns rclcpp::QoS(rclcpp::KeepLast(10))
 * For earlier versions: returns rmw_qos_profile_default
 */
#ifdef ROS_DISTRO_JAZZY
#define AUTOWARE_DEFAULT_TOPIC_QOS_PROFILE() rclcpp::QoS(rclcpp::KeepLast(10))
#else
#define AUTOWARE_DEFAULT_TOPIC_QOS_PROFILE() rmw_qos_profile_default
#endif

}  // namespace autoware::qos_utils

#endif  // AUTOWARE__QOS_UTILS__QOS_COMPATIBILITY_HPP_
