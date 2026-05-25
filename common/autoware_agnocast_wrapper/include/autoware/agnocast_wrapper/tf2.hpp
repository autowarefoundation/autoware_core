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

#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <rclcpp/qos.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/version.h>

#include <memory>
#include <variant>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/static_transform_broadcaster.hpp>
#include <agnocast/node/tf2/transform_broadcaster.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>

namespace autoware::agnocast_wrapper
{

// TODO(Koichi98): agnocast::Buffer currently does not implement waitForTransform, so it has no
// dependency on the agnocast executor — that lets us alias it directly here, which surfaces
// the AgnocastOnly-safety constraint at compile time (the async API simply does not exist
// on the wrapper type). Once waitForTransform is added, this alias must become a
// runtime-dispatched wrapper class.
using Buffer = agnocast::Buffer;

/// @brief TransformListener wrapper; requires a Method 2 node (autoware::agnocast_wrapper::Node).
class TransformListener
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::TransformListener>;
  using AgnocastImpl = std::unique_ptr<agnocast::TransformListener>;

  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS())
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::TransformListener>(
                                                buffer, *node.get_agnocast_node(), spin_thread,
                                                qos, static_qos))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::TransformListener>(
                                              buffer, node.get_rclcpp_node().get(), spin_thread,
                                              qos, static_qos)))
  {
  }

  /// @brief A separate overload, not defaulted options parameters: the no-options constructors
  /// forward nothing, so each backend keeps its own non-trivial option defaults.
  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread, const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos, const AUTOWARE_SUBSCRIPTION_OPTIONS & options,
    const AUTOWARE_SUBSCRIPTION_OPTIONS & static_options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::TransformListener>(
                                                buffer, *node.get_agnocast_node(), spin_thread,
                                                qos, static_qos, options, static_options))
        : [&] {
            rclcpp::SubscriptionOptions ros2_options;
            ros2_options.callback_group = options.callback_group;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
            rclcpp::SubscriptionOptions ros2_static_options;
            ros2_static_options.callback_group = static_options.callback_group;
            ros2_static_options.qos_overriding_options = static_options.qos_overriding_options;
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::TransformListener>(
                                                buffer, node.get_rclcpp_node().get(), spin_thread,
                                                qos, static_qos, ros2_options,
                                                ros2_static_options));
          }())
  {
  }

  /// @brief Buffer-only constructor; the underlying listener creates its own node internally.
  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::TransformListener>(buffer, spin_thread))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>,
            std::make_unique<tf2_ros::TransformListener>(buffer, spin_thread)))
  {
  }

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;
  TransformListener(TransformListener &&) = delete;
  TransformListener & operator=(TransformListener &&) = delete;

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

/// @brief TransformBroadcaster wrapper; requires a Method 2 node
/// (autoware::agnocast_wrapper::Node).
class TransformBroadcaster
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::TransformBroadcaster>;
  using AgnocastImpl = std::unique_ptr<agnocast::TransformBroadcaster>;

  explicit TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS())
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::TransformBroadcaster>(*node.get_agnocast_node(), qos))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>,
            std::make_unique<tf2_ros::TransformBroadcaster>(node.get_rclcpp_node().get(), qos)))
  {
  }

  TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::TransformBroadcaster>(
                                                *node.get_agnocast_node(), qos, options))
        : [&] {
            rclcpp::PublisherOptions ros2_options;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::TransformBroadcaster>(
                                                node.get_rclcpp_node().get(), qos, ros2_options));
          }())
  {
  }

  TransformBroadcaster(const TransformBroadcaster &) = delete;
  TransformBroadcaster & operator=(const TransformBroadcaster &) = delete;
  TransformBroadcaster(TransformBroadcaster &&) = delete;
  TransformBroadcaster & operator=(TransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transform); }, impl_);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transforms); }, impl_);
  }

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

/// @brief StaticTransformBroadcaster wrapper; requires a Method 2 node.
class StaticTransformBroadcaster
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::StaticTransformBroadcaster>;
  using AgnocastImpl = std::unique_ptr<agnocast::StaticTransformBroadcaster>;

  explicit StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::StaticBroadcasterQoS())
  : impl_(
      use_agnocast() ? decltype(impl_)(
                         std::in_place_type<AgnocastImpl>,
                         std::make_unique<agnocast::StaticTransformBroadcaster>(
                           *node.get_agnocast_node(), qos))
                     : decltype(impl_)(
                         std::in_place_type<RclcppImpl>,
                         std::make_unique<tf2_ros::StaticTransformBroadcaster>(
                           node.get_rclcpp_node().get(), qos)))
  {
  }

  StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::StaticTransformBroadcaster>(
                                                *node.get_agnocast_node(), qos, options))
        : [&] {
            rclcpp::PublisherOptions ros2_options;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
#if RCLCPP_VERSION_MAJOR < 28
            // Humble: rclcpp intra-process doesn't support transient_local, so /tf_static needs
            // it off. tf2_ros::StaticTransformBroadcaster did this in its default options
            // pre-Jazzy; Jazzy (rclcpp 28+) removed it.
            ros2_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
#endif
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>,
              std::make_unique<tf2_ros::StaticTransformBroadcaster>(
                node.get_rclcpp_node().get(), qos, ros2_options));
          }())
  {
  }

  StaticTransformBroadcaster(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster & operator=(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster(StaticTransformBroadcaster &&) = delete;
  StaticTransformBroadcaster & operator=(StaticTransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transform); }, impl_);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transforms); }, impl_);
  }

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

// Agnocast-disabled build: thin composition wrappers over the tf2_ros types. Constructor
// signatures match the agnocast-enabled build above.

using Buffer = tf2_ros::Buffer;

class TransformListener
{
public:
  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS())
  : impl_(buffer, &node, spin_thread, qos, static_qos)
  {
  }

  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread, const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos, const AUTOWARE_SUBSCRIPTION_OPTIONS & options,
    const AUTOWARE_SUBSCRIPTION_OPTIONS & static_options)
  : impl_(buffer, &node, spin_thread, qos, static_qos, options, static_options)
  {
  }

  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true)
  : impl_(buffer, spin_thread)
  {
  }

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;
  TransformListener(TransformListener &&) = delete;
  TransformListener & operator=(TransformListener &&) = delete;

private:
  tf2_ros::TransformListener impl_;
};

class TransformBroadcaster
{
public:
  explicit TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS())
  : impl_(&node, qos)
  {
  }

  TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(&node, qos, options)
  {
  }

  TransformBroadcaster(const TransformBroadcaster &) = delete;
  TransformBroadcaster & operator=(const TransformBroadcaster &) = delete;
  TransformBroadcaster(TransformBroadcaster &&) = delete;
  TransformBroadcaster & operator=(TransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    impl_.sendTransform(transform);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    impl_.sendTransform(transforms);
  }

private:
  tf2_ros::TransformBroadcaster impl_;
};

class StaticTransformBroadcaster
{
public:
  explicit StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::StaticBroadcasterQoS())
  : impl_(&node, qos)
  {
  }

  StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(&node, qos, options)
  {
  }

  StaticTransformBroadcaster(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster & operator=(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster(StaticTransformBroadcaster &&) = delete;
  StaticTransformBroadcaster & operator=(StaticTransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    impl_.sendTransform(transform);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    impl_.sendTransform(transforms);
  }

private:
  tf2_ros::StaticTransformBroadcaster impl_;
};

}  // namespace autoware::agnocast_wrapper

#endif
