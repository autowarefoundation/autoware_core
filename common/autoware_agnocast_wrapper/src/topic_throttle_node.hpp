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

#ifndef TOPIC_THROTTLE_NODE_HPP_
#define TOPIC_THROTTLE_NODE_HPP_

#include "autoware/agnocast_wrapper/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// Relays a topic of any type while dropping the messages that arrive before the period implied by
/// msgs_per_sec has elapsed, so the output rate stays at or below it.
///
/// topic_tools::ThrottleNode does the same, but discovers the type and QoS of its input through the
/// ROS 2 graph, which Agnocast does not serve: its NodeGraph::get_publishers_info_by_topic()
/// throws. Both are taken as parameters here instead.
class TopicThrottle : public Node
{
public:
  explicit TopicThrottle(const rclcpp::NodeOptions & options);

private:
  void on_topic(std::shared_ptr<const rclcpp::SerializedMessage> msg);

  std::string topic_;
  std::string remap_topic_;
  std::string topic_type_;

  AUTOWARE_GENERIC_SUBSCRIPTION_PTR sub_topic_;
  AUTOWARE_GENERIC_PUBLISHER_PTR pub_topic_;

  rclcpp::Duration period_;
  rclcpp::Time last_published_time_;
};

}  // namespace autoware::agnocast_wrapper

#endif  // TOPIC_THROTTLE_NODE_HPP_
