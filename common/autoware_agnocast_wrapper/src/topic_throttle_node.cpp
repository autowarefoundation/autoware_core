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

#include "topic_throttle_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::agnocast_wrapper
{

TopicThrottle::TopicThrottle(const rclcpp::NodeOptions & options)
: Node("topic_throttle", options), period_(0, 0)
{
  topic_ = declare_parameter<std::string>("topic");
  remap_topic_ = declare_parameter<std::string>("remap_topic");
  topic_type_ = declare_parameter<std::string>("topic_type");
  const auto msgs_per_sec = declare_parameter<double>("msgs_per_sec");
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto transient_local = declare_parameter<bool>("transient_local", false);
  const auto best_effort = declare_parameter<bool>("best_effort", false);

  if (msgs_per_sec <= 0.0) {
    throw std::invalid_argument("msgs_per_sec must be greater than 0");
  }
  if (qos_depth <= 0) {
    throw std::invalid_argument("qos_depth must be greater than 0");
  }

  period_ = rclcpp::Duration(rclcpp::Rate(msgs_per_sec).period());
  last_published_time_ = now();

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(static_cast<size_t>(qos_depth));
  if (transient_local) {
    qos.transient_local();
  }
  if (best_effort) {
    qos.best_effort();
  }

  pub_topic_ = create_generic_publisher(remap_topic_, topic_type_, qos);
  sub_topic_ = create_generic_subscription(
    topic_, topic_type_, qos,
    [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) { on_topic(std::move(msg)); });
}

void TopicThrottle::on_topic(std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  const auto stamp = now();

  // A clock that jumps back (a replayed bag, a sim-time reset) would otherwise stall the output
  // until the clock caught up with the stale timestamp.
  if (stamp < last_published_time_) {
    RCLCPP_WARN(get_logger(), "Detected jump back in time, resetting the throttle period.");
    last_published_time_ = stamp;
  }

  if (stamp - last_published_time_ < period_) {
    return;
  }

  last_published_time_ = stamp;
  pub_topic_->publish(*msg);
}

}  // namespace autoware::agnocast_wrapper

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::agnocast_wrapper::TopicThrottle)
